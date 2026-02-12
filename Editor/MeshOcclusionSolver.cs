using UnityEngine;
using UnityEditor;
using Unity.Collections;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// Visibility solver combining GPU rendering, multi-hit raycasting,
/// and adjacency expansion for robust occluded face detection.
///
/// Phase 0: GPU rendering from multiple viewpoints (GpuVisibilitySolver)
/// Phase 1: Multi-hit sphere raycasting (4 hits per ray for penetration)
/// Phase 2: Adjacency expansion (N-ring neighbor preservation)
/// </summary>
public static class MeshOcclusionSolver
{
    private const int TempLayer = 31;
    private static readonly int TempLayerMask = 1 << TempLayer;
    private const int BatchSize = 16384;
    private const int MaxHitsPerRay = 4;

    public static Dictionary<MeshFilter, HashSet<int>> SolveVisibility(
        MeshFilter[] meshes, int sphereSamples, int raysPerSample,
        int adjacencyDepth = 1)
    {
        var result = new Dictionary<MeshFilter, HashSet<int>>();
        foreach (var mf in meshes)
            result[mf] = new HashSet<int>();

        // Phase 0: GPU rendering (main detection — most accurate)
        GpuVisibilitySolver.SolveVisibility(meshes, 128, 1024, result);
        LogPhaseResult("Phase 0 (GPU)", meshes, result);

        // Phase 1: Multi-hit sphere raycasting (supplementary)
        var colliderToMesh = new Dictionary<Collider, MeshFilter>();
        var addedColliders = new List<MeshCollider>();
        var disabledColliders = new List<Collider>();
        var originalLayers = new Dictionary<GameObject, int>();

        try
        {
            SetupColliders(meshes, colliderToMesh, addedColliders,
                disabledColliders, originalLayers);
            Physics.SyncTransforms();

            var (center, radius) = ComputeBoundingSphere(meshes);
            float maxDist = radius * 4f;
            var spherePoints = GenerateFibonacciSphere(sphereSamples);

            SphereRaycastBatched(spherePoints, center, radius,
                raysPerSample, maxDist, colliderToMesh, result);
            LogPhaseResult("Phase 1 (Raycast)", meshes, result);
        }
        finally
        {
            CleanupColliders(addedColliders, disabledColliders, originalLayers);
        }

        // Phase 2: Adjacency expansion (safety net)
        if (adjacencyDepth > 0)
        {
            ExpandAdjacency(meshes, result, adjacencyDepth);
            LogPhaseResult("Phase 2 (Adjacency)", meshes, result);
        }

        // Phase 3: Remove small disconnected fragments (conservative)
        RemoveSmallFragments(meshes, result);
        LogPhaseResult("Phase 3 (Fragments)", meshes, result);

        EditorUtility.ClearProgressBar();
        return result;
    }

    // ────────────────────────────────────────────────────────────────
    // Phase 1: Multi-hit Sphere Raycasting
    // ────────────────────────────────────────────────────────────────

    private static void SphereRaycastBatched(
        Vector3[] spherePoints, Vector3 center, float radius,
        int raysPerSample, float maxDist,
        Dictionary<Collider, MeshFilter> colliderToMesh,
        Dictionary<MeshFilter, HashSet<int>> result)
    {
        int raysPerPoint = 1 + raysPerSample;
        int totalRays = spherePoints.Length * raysPerPoint;
        var rayData = new (Vector3 origin, Vector3 dir)[totalRays];

        BuildSphereRays(spherePoints, center, radius, raysPerSample, rayData);

        var queryParams = new QueryParameters(TempLayerMask, false,
            QueryTriggerInteraction.Ignore, false);

        for (int offset = 0; offset < totalRays; offset += BatchSize)
        {
            int count = Mathf.Min(BatchSize, totalRays - offset);
            var commands = new NativeArray<RaycastCommand>(
                count, Allocator.TempJob);
            var hits = new NativeArray<RaycastHit>(
                count * MaxHitsPerRay, Allocator.TempJob);

            try
            {
                FillRaycastCommands(commands, rayData, offset, count,
                    maxDist, queryParams);
                var handle = RaycastCommand.ScheduleBatch(
                    commands, hits, 64, MaxHitsPerRay);
                handle.Complete();
                RecordMultiHits(hits, count, colliderToMesh, result);
            }
            finally
            {
                commands.Dispose();
                hits.Dispose();
            }

            float progress = 0.3f + 0.4f * (offset + count) / (float)totalRays;
            if (EditorUtility.DisplayCancelableProgressBar("Occlusion Analysis",
                $"Phase 1: Multi-hit Raycasting ({offset + count}/{totalRays})",
                progress))
                break;
        }
    }

    private static void BuildSphereRays(
        Vector3[] spherePoints, Vector3 center, float radius,
        int raysPerSample, (Vector3 origin, Vector3 dir)[] rayData)
    {
        float goldenAngle = Mathf.PI * (3f - Mathf.Sqrt(5f));

        for (int i = 0; i < spherePoints.Length; i++)
        {
            Vector3 origin = center + spherePoints[i] * radius;
            Vector3 mainDir = (center - origin).normalized;
            int baseIdx = i * (1 + raysPerSample);
            rayData[baseIdx] = (origin, mainDir);

            Vector3 up = Mathf.Abs(mainDir.y) < 0.999f
                ? Vector3.up : Vector3.right;
            Vector3 tangent = Vector3.Cross(mainDir, up).normalized;
            Vector3 bitangent = Vector3.Cross(mainDir, tangent);

            for (int j = 0; j < raysPerSample; j++)
            {
                float angle = goldenAngle * j;
                float spread = 0.5f * (j + 1f) / raysPerSample;
                Vector3 jitter = tangent * (Mathf.Cos(angle) * spread)
                               + bitangent * (Mathf.Sin(angle) * spread);
                rayData[baseIdx + 1 + j] =
                    (origin, (mainDir + jitter).normalized);
            }
        }
    }

    // ────────────────────────────────────────────────────────────────
    // Phase 2: Adjacency Expansion
    // ────────────────────────────────────────────────────────────────

    /// <summary>
    /// Expand visible set by marking N-ring neighbors of visible triangles.
    /// Prevents holes at the boundary between visible and removed faces.
    /// </summary>
    private static void ExpandAdjacency(
        MeshFilter[] meshes,
        Dictionary<MeshFilter, HashSet<int>> result,
        int depth)
    {
        foreach (var mf in meshes)
        {
            var mesh = mf.sharedMesh;
            if (mesh == null) continue;

            var tris = mesh.triangles;
            int triCount = tris.Length / 3;

            // Build edge-to-triangle adjacency map
            var edgeToTris = new Dictionary<long, List<int>>(triCount * 3);
            for (int t = 0; t < triCount; t++)
            {
                int a = tris[t * 3];
                int b = tris[t * 3 + 1];
                int c = tris[t * 3 + 2];
                AddEdge(edgeToTris, a, b, t);
                AddEdge(edgeToTris, b, c, t);
                AddEdge(edgeToTris, a, c, t);
            }

            // Expand by depth rings
            int beforeCount = result[mf].Count;
            var frontier = new HashSet<int>(result[mf]);

            for (int d = 0; d < depth; d++)
            {
                var nextFrontier = new HashSet<int>();
                foreach (int t in frontier)
                {
                    int a = tris[t * 3];
                    int b = tris[t * 3 + 1];
                    int c = tris[t * 3 + 2];

                    AddNeighbors(edgeToTris, a, b, t, result[mf], nextFrontier);
                    AddNeighbors(edgeToTris, b, c, t, result[mf], nextFrontier);
                    AddNeighbors(edgeToTris, a, c, t, result[mf], nextFrontier);
                }

                if (nextFrontier.Count == 0) break;

                foreach (int t in nextFrontier)
                    result[mf].Add(t);
                frontier = nextFrontier;
            }

            int added = result[mf].Count - beforeCount;
            if (added > 0)
                Debug.Log($"[OcclusionSolver] Adjacency: {mf.gameObject.name} "
                        + $"+{added} triangles (depth {depth})");
        }
    }

    private static void AddEdge(
        Dictionary<long, List<int>> edgeToTris, int v0, int v1, int tri)
    {
        long key = v0 < v1
            ? ((long)v0 << 32) | (uint)v1
            : ((long)v1 << 32) | (uint)v0;

        if (!edgeToTris.TryGetValue(key, out var list))
        {
            list = new List<int>(2);
            edgeToTris[key] = list;
        }
        list.Add(tri);
    }

    private static void AddNeighbors(
        Dictionary<long, List<int>> edgeToTris,
        int v0, int v1, int self,
        HashSet<int> existingVisible, HashSet<int> frontier)
    {
        long key = v0 < v1
            ? ((long)v0 << 32) | (uint)v1
            : ((long)v1 << 32) | (uint)v0;

        if (!edgeToTris.TryGetValue(key, out var list)) return;

        foreach (int neighbor in list)
        {
            if (neighbor != self && !existingVisible.Contains(neighbor))
                frontier.Add(neighbor);
        }
    }

    // ────────────────────────────────────────────────────────────────
    // Phase 3: Small Fragment Removal (Connected Component Filtering)
    // ────────────────────────────────────────────────────────────────

    /// <summary>
    /// Remove small disconnected triangle groups.
    /// Never touches the largest connected component — shape is preserved.
    /// </summary>
    private static void RemoveSmallFragments(
        MeshFilter[] meshes,
        Dictionary<MeshFilter, HashSet<int>> result)
    {
        int totalRemoved = 0;

        foreach (var mf in meshes)
        {
            var mesh = mf.sharedMesh;
            if (mesh == null) continue;
            var visibleSet = result[mf];
            if (visibleSet.Count < 2) continue;

            var tris = mesh.triangles;

            // Build adjacency among visible triangles only
            var edgeToTris = new Dictionary<long, List<int>>();
            foreach (int triIdx in visibleSet)
            {
                int a = tris[triIdx * 3];
                int b = tris[triIdx * 3 + 1];
                int c = tris[triIdx * 3 + 2];
                AddEdge(edgeToTris, a, b, triIdx);
                AddEdge(edgeToTris, b, c, triIdx);
                AddEdge(edgeToTris, a, c, triIdx);
            }

            // BFS to find connected components
            var visited = new HashSet<int>();
            var components = new List<HashSet<int>>();

            foreach (int triIdx in visibleSet)
            {
                if (visited.Contains(triIdx)) continue;

                var component = new HashSet<int>();
                var queue = new Queue<int>();
                queue.Enqueue(triIdx);
                visited.Add(triIdx);

                while (queue.Count > 0)
                {
                    int current = queue.Dequeue();
                    component.Add(current);

                    int a = tris[current * 3];
                    int b = tris[current * 3 + 1];
                    int c = tris[current * 3 + 2];

                    EnqueueNeighbors(edgeToTris, a, b, current, visited, queue);
                    EnqueueNeighbors(edgeToTris, b, c, current, visited, queue);
                    EnqueueNeighbors(edgeToTris, a, c, current, visited, queue);
                }

                components.Add(component);
            }

            if (components.Count <= 1) continue;

            // Find the largest component — never remove it
            int maxIdx = 0;
            for (int i = 1; i < components.Count; i++)
            {
                if (components[i].Count > components[maxIdx].Count)
                    maxIdx = i;
            }

            // Conservative threshold: only remove very small groups
            int minSize = Mathf.Max(5, visibleSet.Count / 100);

            int removed = 0;
            for (int i = 0; i < components.Count; i++)
            {
                if (i == maxIdx) continue; // never touch largest
                if (components[i].Count < minSize)
                {
                    foreach (int triIdx in components[i])
                        visibleSet.Remove(triIdx);
                    removed += components[i].Count;
                }
            }

            if (removed > 0)
            {
                Debug.Log($"[OcclusionSolver] Fragments: {mf.gameObject.name} "
                        + $"removed {removed} tris in small groups "
                        + $"(threshold: {minSize}, kept largest: {components[maxIdx].Count})");
                totalRemoved += removed;
            }
        }

        if (totalRemoved > 0)
            Debug.Log($"[OcclusionSolver] Fragment removal: "
                    + $"{totalRemoved} total triangles removed");
    }

    private static void EnqueueNeighbors(
        Dictionary<long, List<int>> edgeToTris,
        int v0, int v1, int self,
        HashSet<int> visited, Queue<int> queue)
    {
        long key = v0 < v1
            ? ((long)v0 << 32) | (uint)v1
            : ((long)v1 << 32) | (uint)v0;

        if (!edgeToTris.TryGetValue(key, out var list)) return;

        foreach (int neighbor in list)
        {
            if (neighbor != self && !visited.Contains(neighbor))
            {
                visited.Add(neighbor);
                queue.Enqueue(neighbor);
            }
        }
    }

    // ────────────────────────────────────────────────────────────────
    // Shared helpers
    // ────────────────────────────────────────────────────────────────

    private static void FillRaycastCommands(
        NativeArray<RaycastCommand> commands,
        (Vector3 origin, Vector3 dir)[] rayData,
        int offset, int count, float maxDist, QueryParameters queryParams)
    {
        for (int i = 0; i < count; i++)
        {
            var (origin, dir) = rayData[offset + i];
            commands[i] = new RaycastCommand(origin, dir, queryParams, maxDist);
        }
    }

    /// <summary>
    /// Record all hits from multi-hit raycasting.
    /// Each ray can penetrate and hit up to MaxHitsPerRay surfaces.
    /// </summary>
    private static void RecordMultiHits(
        NativeArray<RaycastHit> hits, int rayCount,
        Dictionary<Collider, MeshFilter> colliderToMesh,
        Dictionary<MeshFilter, HashSet<int>> result)
    {
        for (int i = 0; i < rayCount; i++)
        {
            for (int h = 0; h < MaxHitsPerRay; h++)
            {
                var hit = hits[i * MaxHitsPerRay + h];
                if (hit.collider == null) break;
                if (hit.triangleIndex < 0) continue;
                if (!colliderToMesh.TryGetValue(hit.collider, out MeshFilter mf))
                    continue;
                result[mf].Add(hit.triangleIndex);
            }
        }
    }

    // ────────────────────────────────────────────────────────────────
    // Collider setup / cleanup
    // ────────────────────────────────────────────────────────────────

    private static void SetupColliders(
        MeshFilter[] meshes,
        Dictionary<Collider, MeshFilter> colliderToMesh,
        List<MeshCollider> addedColliders,
        List<Collider> disabledColliders,
        Dictionary<GameObject, int> originalLayers)
    {
        foreach (var mf in meshes)
        {
            if (mf.sharedMesh == null) continue;
            var go = mf.gameObject;

            foreach (var existingCollider in go.GetComponents<Collider>())
            {
                if (existingCollider.enabled)
                {
                    existingCollider.enabled = false;
                    disabledColliders.Add(existingCollider);
                }
            }

            if (!originalLayers.ContainsKey(go))
                originalLayers[go] = go.layer;
            go.layer = TempLayer;

            var mc = go.AddComponent<MeshCollider>();
            mc.sharedMesh = mf.sharedMesh;
            addedColliders.Add(mc);
            colliderToMesh[mc] = mf;
        }
    }

    private static void CleanupColliders(
        List<MeshCollider> addedColliders,
        List<Collider> disabledColliders,
        Dictionary<GameObject, int> originalLayers)
    {
        foreach (var mc in addedColliders)
        {
            if (mc != null) Object.DestroyImmediate(mc);
        }
        foreach (var c in disabledColliders)
        {
            if (c != null) c.enabled = true;
        }
        foreach (var kvp in originalLayers)
        {
            if (kvp.Key != null) kvp.Key.layer = kvp.Value;
        }
    }

    // ────────────────────────────────────────────────────────────────
    // Geometry helpers
    // ────────────────────────────────────────────────────────────────

    private static Vector3[] GenerateFibonacciSphere(int samples)
    {
        var points = new Vector3[samples];
        float goldenRatio = (1f + Mathf.Sqrt(5f)) / 2f;

        for (int i = 0; i < samples; i++)
        {
            float theta = 2f * Mathf.PI * i / goldenRatio;
            float phi = Mathf.Acos(1f - 2f * (i + 0.5f) / samples);
            points[i] = new Vector3(
                Mathf.Sin(phi) * Mathf.Cos(theta),
                Mathf.Sin(phi) * Mathf.Sin(theta),
                Mathf.Cos(phi));
        }
        return points;
    }

    private static (Vector3 center, float radius) ComputeBoundingSphere(
        MeshFilter[] meshes)
    {
        var allBounds = meshes
            .Where(mf => mf.sharedMesh != null
                      && mf.GetComponent<Renderer>() != null)
            .Select(mf => mf.GetComponent<Renderer>().bounds)
            .ToArray();

        if (allBounds.Length == 0)
            return (Vector3.zero, 1f);

        var combined = allBounds[0];
        for (int i = 1; i < allBounds.Length; i++)
            combined.Encapsulate(allBounds[i]);

        return (combined.center, combined.extents.magnitude * 1.5f);
    }

    private static void LogPhaseResult(
        string phase, MeshFilter[] meshes,
        Dictionary<MeshFilter, HashSet<int>> result)
    {
        int totalTris = 0;
        int visibleTris = 0;
        foreach (var mf in meshes)
        {
            if (mf.sharedMesh == null) continue;
            var mesh = mf.sharedMesh;
            uint indexCount = 0;
            for (int s = 0; s < mesh.subMeshCount; s++)
                indexCount += mesh.GetIndexCount(s);
            totalTris += (int)(indexCount / 3);
            visibleTris += result[mf].Count;
        }
        float pct = totalTris > 0
            ? (float)visibleTris / totalTris * 100f : 0f;
        Debug.Log($"[OcclusionSolver] {phase}: "
                + $"{visibleTris}/{totalTris} visible ({pct:F1}%)");
    }
}

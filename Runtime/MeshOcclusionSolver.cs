using UnityEngine;
using Unity.Collections;
using System;
using System.Collections.Generic;
using System.Linq;

namespace SlimShredder
{
    /// <summary>
    /// Runtime visibility solver combining GPU rendering, multi-hit raycasting,
    /// and adjacency expansion for robust occluded face detection.
    ///
    /// Phase 0: GPU rendering from multiple viewpoints
    /// Phase 1: Multi-hit sphere raycasting (4 hits per ray)
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
            int adjacencyDepth = 1,
            Action<float, string> onProgress = null)
        {
            var result = new Dictionary<MeshFilter, HashSet<int>>();
            foreach (var mf in meshes)
                result[mf] = new HashSet<int>();

            // Phase 0: GPU rendering (main detection)
            GpuVisibilitySolver.SolveVisibility(meshes, 128, 1024, result, onProgress);
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
                    raysPerSample, maxDist, colliderToMesh, result, onProgress);
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

            return result;
        }

        // ────────────────────────────────────────────────────────────────
        // Phase 1: Multi-hit Sphere Raycasting
        // ────────────────────────────────────────────────────────────────

        private static void SphereRaycastBatched(
            Vector3[] spherePoints, Vector3 center, float radius,
            int raysPerSample, float maxDist,
            Dictionary<Collider, MeshFilter> colliderToMesh,
            Dictionary<MeshFilter, HashSet<int>> result,
            Action<float, string> onProgress)
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
                onProgress?.Invoke(progress,
                    $"Raycasting: {offset + count}/{totalRays}");
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
                if (mc != null) UnityEngine.Object.DestroyImmediate(mc);
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
}

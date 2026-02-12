using UnityEngine;
using UnityEditor;
using Unity.Collections;
using Unity.Jobs;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// Visibility solver using batched RaycastCommands.
/// Phase 1: Sphere raycasting from bounding sphere surface (batched).
/// Phase 2: Reverse ray test — cast from outside toward each unmarked triangle,
///          checking if it is the first surface hit (batched).
/// </summary>
public static class MeshOcclusionSolver
{
    private const int TempLayer = 31;
    private static readonly int TempLayerMask = 1 << TempLayer;
    private const int BatchSize = 16384;

    public static Dictionary<MeshFilter, HashSet<int>> SolveVisibility(
        MeshFilter[] meshes, int sphereSamples, int raysPerSample,
        int hemisphereSamples = 64)
    {
        var result = new Dictionary<MeshFilter, HashSet<int>>();
        foreach (var mf in meshes)
            result[mf] = new HashSet<int>();

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

            // Phase 1: Batched sphere raycasting from bounding sphere surface
            SphereRaycastBatched(spherePoints, center, radius,
                raysPerSample, maxDist, colliderToMesh, result);

            LogPhaseResult("Phase 1", meshes, result);

            // Phase 2: Reverse ray — cast from outside toward each unmarked triangle
            ReverseRayBatched(meshes, result, colliderToMesh,
                hemisphereSamples, radius);

            LogPhaseResult("Phase 2", meshes, result);
        }
        finally
        {
            CleanupColliders(addedColliders, disabledColliders, originalLayers);
            EditorUtility.ClearProgressBar();
        }

        return result;
    }

    // ────────────────────────────────────────────────────────────────
    // Phase 1: Sphere Raycasting
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
            var commands = new NativeArray<RaycastCommand>(count, Allocator.TempJob);
            var hits = new NativeArray<RaycastHit>(count, Allocator.TempJob);

            try
            {
                FillRaycastCommands(commands, rayData, offset, count,
                    maxDist, queryParams);

                var handle = RaycastCommand.ScheduleBatch(commands, hits, 64);
                handle.Complete();

                RecordPhase1Hits(hits, count, colliderToMesh, result);
            }
            finally
            {
                commands.Dispose();
                hits.Dispose();
            }

            float progress = 0.7f * (offset + count) / (float)totalRays;
            if (EditorUtility.DisplayCancelableProgressBar("Occlusion Analysis",
                $"Phase 1: Sphere Raycasting ({offset + count}/{totalRays})",
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
                float spread = 0.15f * (j + 1f) / raysPerSample;
                Vector3 jitter = tangent * (Mathf.Cos(angle) * spread)
                               + bitangent * (Mathf.Sin(angle) * spread);
                rayData[baseIdx + 1 + j] = (origin, (mainDir + jitter).normalized);
            }
        }
    }

    // ────────────────────────────────────────────────────────────────
    // Phase 2: Reverse Ray Visibility Test
    // For each unmarked triangle, cast rays FROM outside TOWARD its center.
    // If the first hit IS that triangle, it is visible from that direction.
    // This correctly handles concave surfaces (slots, grooves, recesses).
    // ────────────────────────────────────────────────────────────────

    private static void ReverseRayBatched(
        MeshFilter[] meshes,
        Dictionary<MeshFilter, HashSet<int>> result,
        Dictionary<Collider, MeshFilter> colliderToMesh,
        int hemisphereSamples, float radius)
    {
        var unmarked = CollectUnmarkedTriangles(meshes, result);
        if (unmarked.Count == 0) return;

        // Build reverse lookup: MeshFilter → its temp MeshCollider
        var meshToCollider = new Dictionary<MeshFilter, Collider>();
        foreach (var kvp in colliderToMesh)
            meshToCollider[kvp.Value] = kvp.Key;

        // Hemisphere directions oriented around each triangle's normal
        var allDirs = GenerateFibonacciSphere(hemisphereSamples);
        var validDirs = allDirs.Where(d => d.y >= 0.05f).ToArray();
        int raysPerTri = validDirs.Length;
        int totalRays = unmarked.Count * raysPerTri;
        float maxDist = radius * 1.5f;

        Debug.Log($"[OcclusionSolver] Phase 2 (Reverse Ray): {unmarked.Count} "
                + $"triangles × {raysPerTri} dirs = {totalRays} rays");

        var queryParams = new QueryParameters(TempLayerMask, false,
            QueryTriggerInteraction.Ignore, false);
        var visibleIndices = new HashSet<int>();

        for (int rayOffset = 0; rayOffset < totalRays; rayOffset += BatchSize)
        {
            int count = Mathf.Min(BatchSize, totalRays - rayOffset);
            var commands = new NativeArray<RaycastCommand>(count, Allocator.TempJob);
            var hits = new NativeArray<RaycastHit>(count, Allocator.TempJob);

            try
            {
                FillReverseRayCommands(commands, count, unmarked,
                    validDirs, raysPerTri, rayOffset, radius,
                    maxDist, queryParams);

                var handle = RaycastCommand.ScheduleBatch(commands, hits, 64);
                handle.Complete();

                // A hit that matches the expected (collider, triangleIndex)
                // means that triangle is directly visible from that direction
                for (int i = 0; i < count; i++)
                {
                    if (hits[i].collider == null || hits[i].triangleIndex < 0)
                        continue;

                    int triLocal = (rayOffset + i) / raysPerTri;
                    if (visibleIndices.Contains(triLocal)) continue;

                    var (mf, expectedTriIdx, _, _) = unmarked[triLocal];

                    if (meshToCollider.TryGetValue(mf, out var expectedCollider)
                        && hits[i].collider == expectedCollider
                        && hits[i].triangleIndex == expectedTriIdx)
                    {
                        visibleIndices.Add(triLocal);
                    }
                }
            }
            finally
            {
                commands.Dispose();
                hits.Dispose();
            }

            float progress = 0.7f + 0.3f * (rayOffset + count) / (float)totalRays;
            if (EditorUtility.DisplayCancelableProgressBar("Occlusion Analysis",
                $"Phase 2: Reverse Ray ({rayOffset + count}/{totalRays})",
                progress))
                break;
        }

        foreach (int idx in visibleIndices)
        {
            var (mf, triIdx, _, _) = unmarked[idx];
            result[mf].Add(triIdx);
        }
    }

    /// <summary>
    /// Build reverse ray commands: origin is placed at (triCenter + dir * radius),
    /// ray direction points back toward triCenter (-dir).
    /// </summary>
    private static void FillReverseRayCommands(
        NativeArray<RaycastCommand> commands, int count,
        List<(MeshFilter mf, int triIdx, Vector3 center, Vector3 normal)> unmarked,
        Vector3[] validDirs, int raysPerTri,
        int rayStartOffset, float testDist,
        float maxDist, QueryParameters queryParams)
    {
        for (int i = 0; i < count; i++)
        {
            int globalRay = rayStartOffset + i;
            int triLocal = globalRay / raysPerTri;
            int dirIdx = globalRay % raysPerTri;

            var (_, _, center, normal) = unmarked[triLocal];

            Vector3 up = Mathf.Abs(normal.y) < 0.999f
                ? Vector3.up : Vector3.right;
            Vector3 tangent = Vector3.Cross(normal, up).normalized;
            Vector3 bitangent = Vector3.Cross(normal, tangent);

            Vector3 localDir = validDirs[dirIdx];
            Vector3 worldDir = tangent * localDir.x
                             + normal * localDir.y
                             + bitangent * localDir.z;

            // Origin: far from triangle along this direction
            // Ray: points back toward the triangle center
            Vector3 origin = center + worldDir * testDist;
            commands[i] = new RaycastCommand(origin, -worldDir,
                queryParams, maxDist);
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

    private static void RecordPhase1Hits(
        NativeArray<RaycastHit> hits, int count,
        Dictionary<Collider, MeshFilter> colliderToMesh,
        Dictionary<MeshFilter, HashSet<int>> result)
    {
        for (int i = 0; i < count; i++)
        {
            var hit = hits[i];
            if (hit.collider == null) continue;
            if (hit.triangleIndex < 0) continue;
            if (!colliderToMesh.TryGetValue(hit.collider, out MeshFilter mf))
                continue;
            result[mf].Add(hit.triangleIndex);
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

    private static List<(MeshFilter mf, int triIdx, Vector3 center, Vector3 normal)>
        CollectUnmarkedTriangles(
            MeshFilter[] meshes,
            Dictionary<MeshFilter, HashSet<int>> result)
    {
        var unmarked = new List<(MeshFilter, int, Vector3, Vector3)>();

        foreach (var mf in meshes)
        {
            var mesh = mf.sharedMesh;
            if (mesh == null) continue;

            var tris = mesh.triangles;
            var verts = mesh.vertices;
            var tf = mf.transform;
            int triCount = tris.Length / 3;

            for (int t = 0; t < triCount; t++)
            {
                if (result[mf].Contains(t)) continue;

                Vector3 v0 = tf.TransformPoint(verts[tris[t * 3]]);
                Vector3 v1 = tf.TransformPoint(verts[tris[t * 3 + 1]]);
                Vector3 v2 = tf.TransformPoint(verts[tris[t * 3 + 2]]);

                Vector3 cross = Vector3.Cross(v1 - v0, v2 - v0);
                if (cross.sqrMagnitude < 1e-10f) continue;

                unmarked.Add((mf, t, (v0 + v1 + v2) / 3f, cross.normalized));
            }
        }
        return unmarked;
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
            totalTris += mf.sharedMesh.triangles.Length / 3;
            visibleTris += result[mf].Count;
        }
        float pct = totalTris > 0
            ? (float)visibleTris / totalTris * 100f : 0f;
        Debug.Log($"[OcclusionSolver] {phase}: "
                + $"{visibleTris}/{totalTris} visible ({pct:F1}%)");
    }
}

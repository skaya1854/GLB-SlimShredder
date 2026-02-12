using UnityEngine;
using UnityEditor;
using Unity.Collections;
using Unity.Jobs;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// Visibility solver using batched RaycastCommands for performance.
/// Phase 1: Sphere raycasting from bounding sphere surface (batched).
/// Phase 2: Hemisphere raycasting from unmarked triangle centers (multi-direction).
/// </summary>
public static class MeshOcclusionSolver
{
    private const int TempLayer = 31;
    private static readonly int TempLayerMask = 1 << TempLayer;
    private const int BatchSize = 16384;
    private const float NormalOffset = 0.01f;

    public static Dictionary<MeshFilter, HashSet<int>> SolveVisibility(
        MeshFilter[] meshes, int sphereSamples, int raysPerSample,
        int hemisphereSamples = 32)
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
            var spherePoints = GenerateFibonacciSphere(sphereSamples);

            // Phase 1: Batched sphere raycasting
            SphereRaycastBatched(spherePoints, center, radius,
                raysPerSample, colliderToMesh, result);

            // Phase 2: Hemisphere raycasting for unmarked triangles
            HemisphereRaycast(meshes, result, hemisphereSamples);
        }
        finally
        {
            CleanupColliders(addedColliders, disabledColliders, originalLayers);
            EditorUtility.ClearProgressBar();
        }

        return result;
    }

    /// <summary>
    /// Phase 1: Build all sphere rays, execute in batches via RaycastCommand.
    /// </summary>
    private static void SphereRaycastBatched(
        Vector3[] spherePoints, Vector3 center, float radius,
        int raysPerSample, Dictionary<Collider, MeshFilter> colliderToMesh,
        Dictionary<MeshFilter, HashSet<int>> result)
    {
        int raysPerPoint = 1 + raysPerSample;
        int totalRays = spherePoints.Length * raysPerPoint;
        var rayData = new (Vector3 origin, Vector3 dir)[totalRays];

        BuildSphereRays(spherePoints, center, radius, raysPerSample, rayData);
        ExecuteBatchedRays(rayData, radius * 3f, colliderToMesh, result,
            "Phase 1: Sphere Raycasting", 0f, 0.7f);
    }

    /// <summary>
    /// Fill ray origin/direction pairs for sphere raycasting with stratified jitter.
    /// </summary>
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

            // Stratified jitter using golden angle spiral on tangent plane
            Vector3 tangent = Vector3.Cross(mainDir, Vector3.up).normalized;
            if (tangent.sqrMagnitude < 0.01f)
                tangent = Vector3.Cross(mainDir, Vector3.right).normalized;
            Vector3 bitangent = Vector3.Cross(mainDir, tangent);

            for (int j = 0; j < raysPerSample; j++)
            {
                float angle = goldenAngle * j;
                float spread = 0.15f * (j + 1f) / raysPerSample;
                Vector3 offset = tangent * (Mathf.Cos(angle) * spread)
                               + bitangent * (Mathf.Sin(angle) * spread);
                rayData[baseIdx + 1 + j] = (origin, (mainDir + offset).normalized);
            }
        }
    }

    /// <summary>
    /// Phase 2: For each unmarked triangle, cast rays across a hemisphere to test visibility
    /// from multiple directions, not just the face normal.
    /// </summary>
    private static void HemisphereRaycast(
        MeshFilter[] meshes, Dictionary<MeshFilter, HashSet<int>> result,
        int hemisphereSamples)
    {
        var unmarked = CollectUnmarkedTriangles(meshes, result);
        if (unmarked.Count == 0) return;

        var hemiDirs = GenerateFibonacciSphere(hemisphereSamples);

        for (int i = 0; i < unmarked.Count; i++)
        {
            var (mf, triIdx, center, normal) = unmarked[i];

            if (TestHemisphereVisibility(center, normal, hemiDirs))
                result[mf].Add(triIdx);

            if (i % 512 == 0)
            {
                float progress = 0.7f + 0.3f * i / unmarked.Count;
                if (EditorUtility.DisplayCancelableProgressBar(
                    "Occlusion Analysis",
                    $"Phase 2: Hemisphere Raycasting ({i}/{unmarked.Count})",
                    progress))
                    break;
            }
        }
    }

    /// <summary>
    /// Test if a triangle is visible by casting rays across the hemisphere oriented along its normal.
    /// Returns true if ANY ray escapes without hitting geometry.
    /// </summary>
    private static bool TestHemisphereVisibility(
        Vector3 center, Vector3 normal, Vector3[] hemiDirs)
    {
        // Build local coordinate frame around the normal
        Vector3 tangent = Vector3.Cross(normal, Vector3.up).normalized;
        if (tangent.sqrMagnitude < 0.01f)
            tangent = Vector3.Cross(normal, Vector3.right).normalized;
        Vector3 bitangent = Vector3.Cross(normal, tangent);

        for (int d = 0; d < hemiDirs.Length; d++)
        {
            Vector3 localDir = hemiDirs[d];

            // Only use directions in the upper hemisphere (aligned with normal)
            if (localDir.y < 0.05f) continue;

            // Transform from unit hemisphere to world orientation
            Vector3 worldDir = tangent * localDir.x
                             + normal * localDir.y
                             + bitangent * localDir.z;

            Vector3 origin = center + worldDir * NormalOffset;

            if (!Physics.Raycast(new Ray(origin, worldDir), out _, Mathf.Infinity, TempLayerMask))
                return true;
        }

        return false;
    }

    /// <summary>
    /// Execute rays in batches using RaycastCommand for parallel processing on the Job System.
    /// </summary>
    private static void ExecuteBatchedRays(
        (Vector3 origin, Vector3 dir)[] rayData, float maxDist,
        Dictionary<Collider, MeshFilter> colliderToMesh,
        Dictionary<MeshFilter, HashSet<int>> result,
        string progressLabel, float progressMin, float progressMax)
    {
        var queryParams = new QueryParameters(TempLayerMask, false, QueryTriggerInteraction.Ignore);

        for (int offset = 0; offset < rayData.Length; offset += BatchSize)
        {
            int count = Mathf.Min(BatchSize, rayData.Length - offset);

            var commands = new NativeArray<RaycastCommand>(count, Allocator.TempJob);
            var hits = new NativeArray<RaycastHit>(count, Allocator.TempJob);

            try
            {
                FillRaycastCommands(commands, rayData, offset, count, maxDist, queryParams);

                var handle = RaycastCommand.ScheduleBatch(commands, hits, 64);
                handle.Complete();

                RecordBatchHits(hits, count, colliderToMesh, result);
            }
            finally
            {
                commands.Dispose();
                hits.Dispose();
            }

            float progress = progressMin + (progressMax - progressMin) *
                (offset + count) / (float)rayData.Length;
            if (EditorUtility.DisplayCancelableProgressBar("Occlusion Analysis",
                $"{progressLabel} ({offset + count}/{rayData.Length})", progress))
                break;
        }
    }

    /// <summary>
    /// Fill NativeArray with RaycastCommands from ray data.
    /// </summary>
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
    /// Process batch raycast results and record visible triangles.
    /// </summary>
    private static void RecordBatchHits(
        NativeArray<RaycastHit> hits, int count,
        Dictionary<Collider, MeshFilter> colliderToMesh,
        Dictionary<MeshFilter, HashSet<int>> result)
    {
        for (int i = 0; i < count; i++)
        {
            var hit = hits[i];
            if (hit.collider == null) continue;
            if (hit.triangleIndex < 0) continue;
            if (!colliderToMesh.TryGetValue(hit.collider, out MeshFilter mf)) continue;

            result[mf].Add(hit.triangleIndex);
        }
    }

    /// <summary>
    /// Setup temp colliders: DISABLE existing colliders to prevent ray interception,
    /// then add our own MeshColliders on layer 31.
    /// </summary>
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

            // Disable ALL existing colliders so they don't intercept our rays
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

    /// <summary>
    /// Remove temp colliders, re-enable disabled colliders, restore layers.
    /// </summary>
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
        CollectUnmarkedTriangles(MeshFilter[] meshes, Dictionary<MeshFilter, HashSet<int>> result)
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

                Vector3 center = (v0 + v1 + v2) / 3f;
                Vector3 normal = Vector3.Cross(v1 - v0, v2 - v0).normalized;
                unmarked.Add((mf, t, center, normal));
            }
        }

        return unmarked;
    }

    private static (Vector3 center, float radius) ComputeBoundingSphere(MeshFilter[] meshes)
    {
        var allBounds = meshes
            .Where(mf => mf.sharedMesh != null && mf.GetComponent<Renderer>() != null)
            .Select(mf => mf.GetComponent<Renderer>().bounds)
            .ToArray();

        if (allBounds.Length == 0)
            return (Vector3.zero, 1f);

        var combined = allBounds[0];
        for (int i = 1; i < allBounds.Length; i++)
            combined.Encapsulate(allBounds[i]);

        return (combined.center, combined.extents.magnitude * 1.5f);
    }
}

using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// Core algorithm for determining triangle visibility using sphere raycasting and normal raycasting.
/// </summary>
public static class MeshOcclusionSolver
{
    private const int TempLayer = 31;
    private static readonly int TempLayerMask = 1 << TempLayer;

    /// <summary>
    /// Main entry: returns visible triangle indices per MeshFilter.
    /// Phase 1: Sphere raycasting from bounding sphere surface.
    /// Phase 2: Normal raycasting from triangle centers outward.
    /// </summary>
    public static Dictionary<MeshFilter, HashSet<int>> SolveVisibility(
        MeshFilter[] meshes, int sphereSamples, int raysPerSample)
    {
        var result = new Dictionary<MeshFilter, HashSet<int>>();
        foreach (var mf in meshes)
            result[mf] = new HashSet<int>();

        // Build collider-to-meshfilter mapping and setup temp colliders
        var colliderToMesh = new Dictionary<Collider, MeshFilter>();
        var addedColliders = new List<MeshCollider>();
        var originalLayers = new Dictionary<GameObject, int>();

        try
        {
            SetupTempColliders(meshes, colliderToMesh, addedColliders, originalLayers);

            // Sync physics scene so raycasts work
            Physics.SyncTransforms();

            // Calculate bounding sphere
            var (sphereCenter, sphereRadius) = ComputeBoundingSphere(meshes);

            // Phase 1: Sphere raycasting
            var spherePoints = GenerateFibonacciSphere(sphereSamples);
            SphereRaycast(spherePoints, sphereCenter, sphereRadius,
                raysPerSample, colliderToMesh, result);

            // Phase 2: Normal raycasting for remaining triangles
            NormalRaycast(meshes, result, colliderToMesh);
        }
        finally
        {
            CleanupTempColliders(addedColliders, originalLayers);
            EditorUtility.ClearProgressBar();
        }

        return result;
    }

    /// <summary>
    /// Generate evenly distributed points on a unit sphere using Fibonacci spiral.
    /// </summary>
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
                Mathf.Cos(phi)
            );
        }

        return points;
    }

    /// <summary>
    /// Phase 1: Cast rays from sphere surface toward the mesh center with jittered spread.
    /// </summary>
    private static void SphereRaycast(
        Vector3[] spherePoints, Vector3 center, float radius,
        int raysPerSample, Dictionary<Collider, MeshFilter> colliderToMesh,
        Dictionary<MeshFilter, HashSet<int>> result)
    {
        for (int i = 0; i < spherePoints.Length; i++)
        {
            Vector3 origin = center + spherePoints[i] * radius;
            Vector3 mainDir = (center - origin).normalized;

            // Main ray toward center
            CastAndRecord(origin, mainDir, radius * 3f, colliderToMesh, result);

            // Jittered rays for better coverage
            for (int j = 0; j < raysPerSample; j++)
            {
                Vector3 jitter = Random.insideUnitSphere * 0.05f;
                Vector3 jitteredDir = (mainDir + jitter).normalized;
                CastAndRecord(origin, jitteredDir, radius * 3f, colliderToMesh, result);
            }

            // Progress bar: Phase 1 occupies 0 ~ 0.7
            if (i % 64 == 0)
            {
                float progress = 0.7f * i / spherePoints.Length;
                if (EditorUtility.DisplayCancelableProgressBar(
                    "Occlusion Analysis", $"Phase 1: Sphere Raycasting ({i}/{spherePoints.Length})", progress))
                    break;
            }
        }
    }

    /// <summary>
    /// Phase 2: Cast rays from triangle centers along their normals.
    /// Triangles not hit in Phase 1 are tested — if ray escapes without hitting anything, the face is visible.
    /// </summary>
    private static void NormalRaycast(
        MeshFilter[] meshes, Dictionary<MeshFilter, HashSet<int>> result,
        Dictionary<Collider, MeshFilter> colliderToMesh)
    {
        var unmarked = CollectUnmarkedTriangles(meshes, result);

        for (int i = 0; i < unmarked.Count; i++)
        {
            var (mf, triIdx, center, normal) = unmarked[i];

            // Offset origin along normal to avoid self-hit (0.01 prevents float precision issues)
            Vector3 origin = center + normal * 0.01f;

            // If ray doesn't hit anything, the face looks outward (visible)
            if (!Physics.Raycast(new Ray(origin, normal), out _, Mathf.Infinity, TempLayerMask))
                result[mf].Add(triIdx);

            // Progress bar: Phase 2 occupies 0.7 ~ 1.0
            if (i % 256 == 0)
            {
                float progress = 0.7f + 0.3f * i / unmarked.Count;
                if (EditorUtility.DisplayCancelableProgressBar(
                    "Occlusion Analysis", $"Phase 2: Normal Raycasting ({i}/{unmarked.Count})", progress))
                    break;
            }
        }
    }

    /// <summary>
    /// Collect triangles that were not marked visible in Phase 1 with their world-space center and normal.
    /// </summary>
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

                var (c, n) = GetTriangleInfo(v0, v1, v2);
                unmarked.Add((mf, t, c, n));
            }
        }

        return unmarked;
    }

    /// <summary>
    /// Compute the center and outward normal of a triangle from its three world-space vertices.
    /// </summary>
    private static (Vector3 center, Vector3 normal) GetTriangleInfo(
        Vector3 v0, Vector3 v1, Vector3 v2)
    {
        Vector3 center = (v0 + v1 + v2) / 3f;
        Vector3 normal = Vector3.Cross(v1 - v0, v2 - v0).normalized;
        return (center, normal);
    }

    /// <summary>
    /// Cast a single ray and record any hit triangle as visible.
    /// </summary>
    private static void CastAndRecord(
        Vector3 origin, Vector3 direction, float maxDist,
        Dictionary<Collider, MeshFilter> colliderToMesh,
        Dictionary<MeshFilter, HashSet<int>> result)
    {
        if (!Physics.Raycast(new Ray(origin, direction), out RaycastHit hit, maxDist, TempLayerMask))
            return;

        if (!colliderToMesh.TryGetValue(hit.collider, out MeshFilter mf))
            return;

        if (hit.triangleIndex >= 0)
            result[mf].Add(hit.triangleIndex);
    }

    /// <summary>
    /// Add temporary MeshColliders on layer 31 for raycasting, preserving original layers.
    /// </summary>
    private static void SetupTempColliders(
        MeshFilter[] meshes,
        Dictionary<Collider, MeshFilter> colliderToMesh,
        List<MeshCollider> addedColliders,
        Dictionary<GameObject, int> originalLayers)
    {
        foreach (var mf in meshes)
        {
            if (mf.sharedMesh == null) continue;

            var go = mf.gameObject;

            // Preserve original layer
            if (!originalLayers.ContainsKey(go))
                originalLayers[go] = go.layer;
            go.layer = TempLayer;

            // Add temporary MeshCollider
            var mc = go.AddComponent<MeshCollider>();
            mc.sharedMesh = mf.sharedMesh;
            addedColliders.Add(mc);
            colliderToMesh[mc] = mf;
        }
    }

    /// <summary>
    /// Remove all temporary MeshColliders and restore original layers.
    /// </summary>
    private static void CleanupTempColliders(
        List<MeshCollider> addedColliders,
        Dictionary<GameObject, int> originalLayers)
    {
        foreach (var mc in addedColliders)
        {
            if (mc != null)
                Object.DestroyImmediate(mc);
        }

        foreach (var kvp in originalLayers)
        {
            if (kvp.Key != null)
                kvp.Key.layer = kvp.Value;
        }
    }

    /// <summary>
    /// Compute the bounding sphere that encloses all mesh bounds, expanded by 1.5x radius.
    /// </summary>
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

        float radius = combined.extents.magnitude * 1.5f;
        return (combined.center, radius);
    }
}

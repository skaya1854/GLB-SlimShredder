using UnityEngine;
using UnityEngine.Rendering;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// GPU-based visibility detection using CommandBuffer rendering.
/// Renders triangle IDs as vertex colors from multiple viewpoints,
/// reads back pixels to determine which triangles are visible.
/// </summary>
public static class GpuVisibilitySolver
{
    /// <summary>
    /// Detect visible triangles by rendering from multiple viewpoints.
    /// Triangle IDs are encoded as vertex colors (24-bit RGB).
    /// </summary>
    public static void SolveVisibility(
        MeshFilter[] meshes, int viewpointCount, int resolution,
        Dictionary<MeshFilter, HashSet<int>> result,
        int minViewpoints = 1)
    {
        // Build ID-encoded meshes and mapping
        var idMeshes = new List<(Mesh mesh, Matrix4x4 matrix)>();
        var meshMapping = new List<(MeshFilter mf, int triOffset, int triCount)>();
        int globalOffset = 0;

        foreach (var mf in meshes)
        {
            if (mf.sharedMesh == null) continue;
            int triCount = CountTriangles(mf.sharedMesh);
            if (triCount == 0) continue;

            var idMesh = BuildIDMesh(mf.sharedMesh, globalOffset);
            idMeshes.Add((idMesh, mf.transform.localToWorldMatrix));
            meshMapping.Add((mf, globalOffset, triCount));
            globalOffset += triCount;
        }

        if (idMeshes.Count == 0) return;

        // Render resources: float RT avoids sRGB precision loss
        var desc = new RenderTextureDescriptor(
            resolution, resolution, RenderTextureFormat.ARGBFloat, 24);
        desc.sRGB = false;
        desc.msaaSamples = 1;
        var rt = new RenderTexture(desc);
        rt.filterMode = FilterMode.Point;
        rt.Create();

        var tex = new Texture2D(
            resolution, resolution, TextureFormat.RGBAFloat, false, true);
        var mat = CreateIDMaterial();
        var cmd = new CommandBuffer { name = "TriangleIDRender" };

        // Virtual camera for view/projection matrices
        var camGo = new GameObject("_TempVisibilityCam");
        camGo.hideFlags = HideFlags.HideAndDontSave;
        var cam = camGo.AddComponent<Camera>();
        cam.fieldOfView = 90f;
        cam.aspect = 1f;
        cam.nearClipPlane = 0.001f;

        var (center, radius) = ComputeBoundingSphere(meshes);
        cam.farClipPlane = radius * 4f;

        var viewpoints = GenerateFibonacciSphere(viewpointCount);

        try
        {
            int totalTris = globalOffset;
            int totalVisible = 0;
            int meshCount = idMeshes.Count;
            var sw = System.Diagnostics.Stopwatch.StartNew();

            // Per-triangle viewpoint vote counter (globalTriIdx -> count)
            var viewCounts = minViewpoints > 1
                ? new Dictionary<int, int>() : null;

            Debug.Log($"[GpuVisibility] Start: {meshCount} meshes, "
                    + $"{totalTris:N0} triangles, "
                    + $"{viewpoints.Length} viewpoints @ {resolution}x{resolution}"
                    + (minViewpoints > 1
                        ? $", minViewpoints={minViewpoints}" : ""));

            for (int v = 0; v < viewpoints.Length; v++)
            {
                camGo.transform.position = center + viewpoints[v] * radius;
                camGo.transform.LookAt(center);

                cmd.Clear();
                cmd.SetRenderTarget(rt);
                cmd.ClearRenderTarget(true, true, Color.black);
                cmd.SetViewProjectionMatrices(
                    cam.worldToCameraMatrix, cam.projectionMatrix);

                foreach (var (mesh, worldMatrix) in idMeshes)
                    cmd.DrawMesh(mesh, worldMatrix, mat, 0, 0);

                Graphics.ExecuteCommandBuffer(cmd);

                RenderTexture.active = rt;
                tex.ReadPixels(new Rect(0, 0, resolution, resolution), 0, 0);
                tex.Apply();
                RenderTexture.active = null;

                // Collect per-viewpoint unique global IDs for vote counting
                HashSet<int> seenThisView = viewCounts != null
                    ? new HashSet<int>() : null;

                int found = DecodePixels(tex, meshMapping, result, seenThisView);
                totalVisible += found;

                // Increment vote counts for each triangle seen this viewpoint
                if (seenThisView != null)
                {
                    foreach (int globalIdx in seenThisView)
                    {
                        viewCounts.TryGetValue(globalIdx, out int c);
                        viewCounts[globalIdx] = c + 1;
                    }
                }

                int cumVisible = result.Values.Sum(s => s.Count);
                float visPct = totalTris > 0
                    ? (float)cumVisible / totalTris * 100f : 0f;

                if (EditorUtility.DisplayCancelableProgressBar(
                    "GPU Visibility",
                    $"Viewpoint {v + 1}/{viewpoints.Length} | "
                    + $"Visible: {cumVisible:N0}/{totalTris:N0} ({visPct:F1}%) | "
                    + $"+{found} new | {sw.Elapsed.TotalSeconds:F1}s",
                    (float)(v + 1) / viewpoints.Length * 0.3f))
                    break;
            }

            // Filter by minimum viewpoint votes
            if (viewCounts != null)
            {
                int beforeFilter = result.Values.Sum(s => s.Count);
                int removed = 0;

                foreach (var (mf, triOffset, triCount) in meshMapping)
                {
                    var visible = result[mf];
                    var toRemove = new List<int>();
                    foreach (int localTri in visible)
                    {
                        int globalIdx = triOffset + localTri;
                        if (!viewCounts.TryGetValue(globalIdx, out int count)
                            || count < minViewpoints)
                            toRemove.Add(localTri);
                    }
                    foreach (int tri in toRemove)
                        visible.Remove(tri);
                    removed += toRemove.Count;
                }

                Debug.Log($"[GpuVisibility] Voting filter (min={minViewpoints}): "
                        + $"removed {removed:N0} low-confidence triangles "
                        + $"({beforeFilter:N0} -> {beforeFilter - removed:N0})");
            }

            sw.Stop();
            int finalVisible = result.Values.Sum(s => s.Count);
            float finalPct = totalTris > 0
                ? (float)finalVisible / totalTris * 100f : 0f;

            Debug.Log($"[GpuVisibility] Done in {sw.Elapsed.TotalSeconds:F1}s: "
                    + $"{finalVisible:N0}/{totalTris:N0} visible ({finalPct:F1}%)");
        }
        finally
        {
            cmd.Dispose();
            Object.DestroyImmediate(camGo);
            Object.DestroyImmediate(rt);
            Object.DestroyImmediate(tex);
            Object.DestroyImmediate(mat);
            foreach (var (mesh, _) in idMeshes)
                Object.DestroyImmediate(mesh);
        }
    }

    // ────────────────────────────────────────────────────────────────
    // ID Mesh Construction
    // ────────────────────────────────────────────────────────────────

    /// <summary>
    /// Build exploded mesh: each triangle gets unique vertices with
    /// vertex color encoding the global triangle ID (1-indexed, 0=background).
    /// </summary>
    private static Mesh BuildIDMesh(Mesh original, int globalTriOffset)
    {
        var srcVerts = original.vertices;
        var srcTris = original.triangles;
        int triCount = srcTris.Length / 3;

        var newVerts = new Vector3[triCount * 3];
        var newColors = new Color32[triCount * 3];
        var newTris = new int[triCount * 3];

        for (int t = 0; t < triCount; t++)
        {
            int globalId = globalTriOffset + t + 1;
            var idColor = EncodeID(globalId);

            for (int k = 0; k < 3; k++)
            {
                int idx = t * 3 + k;
                newVerts[idx] = srcVerts[srcTris[idx]];
                newColors[idx] = idColor;
                newTris[idx] = idx;
            }
        }

        var mesh = new Mesh { name = "IDMesh_" + original.name };
        if (newVerts.Length > 65535)
            mesh.indexFormat = IndexFormat.UInt32;
        mesh.vertices = newVerts;
        mesh.colors32 = newColors;
        mesh.triangles = newTris;
        mesh.RecalculateBounds();
        return mesh;
    }

    // ────────────────────────────────────────────────────────────────
    // ID Encoding / Decoding
    // ────────────────────────────────────────────────────────────────

    private static Color32 EncodeID(int id)
    {
        return new Color32(
            (byte)((id >> 16) & 0xFF),
            (byte)((id >> 8) & 0xFF),
            (byte)(id & 0xFF),
            255);
    }

    private static int DecodeID(Color pixel)
    {
        int r = Mathf.RoundToInt(pixel.r * 255f);
        int g = Mathf.RoundToInt(pixel.g * 255f);
        int b = Mathf.RoundToInt(pixel.b * 255f);
        return (r << 16) | (g << 8) | b;
    }

    /// <summary>
    /// Decode visible triangle IDs from rendered texture.
    /// Returns number of newly discovered triangles.
    /// </summary>
    private static int DecodePixels(
        Texture2D tex,
        List<(MeshFilter mf, int triOffset, int triCount)> meshMapping,
        Dictionary<MeshFilter, HashSet<int>> result,
        HashSet<int> seenGlobalIds = null)
    {
        var pixels = tex.GetPixels();
        int newCount = 0;

        foreach (var pixel in pixels)
        {
            int globalId = DecodeID(pixel);
            if (globalId <= 0) continue;

            int triIdx = globalId - 1;
            seenGlobalIds?.Add(triIdx);

            foreach (var (mf, triOffset, triCount) in meshMapping)
            {
                if (triIdx >= triOffset && triIdx < triOffset + triCount)
                {
                    if (result[mf].Add(triIdx - triOffset))
                        newCount++;
                    break;
                }
            }
        }

        return newCount;
    }

    // ────────────────────────────────────────────────────────────────
    // Material
    // ────────────────────────────────────────────────────────────────

    private static Material CreateIDMaterial()
    {
        var shader = Shader.Find("Hidden/Internal-Colored");
        if (shader == null)
        {
            Debug.LogError("[GpuVisibility] Hidden/Internal-Colored shader not found");
            return new Material(Shader.Find("Sprites/Default"));
        }

        var mat = new Material(shader);
        mat.SetInt("_SrcBlend", (int)BlendMode.One);
        mat.SetInt("_DstBlend", (int)BlendMode.Zero);
        mat.SetInt("_Cull", (int)CullMode.Back);
        mat.SetInt("_ZWrite", 1);
        mat.SetInt("_ZTest", (int)CompareFunction.LessEqual);
        return mat;
    }

    // ────────────────────────────────────────────────────────────────
    // Geometry helpers
    // ────────────────────────────────────────────────────────────────

    private static int CountTriangles(Mesh mesh)
    {
        uint total = 0;
        for (int i = 0; i < mesh.subMeshCount; i++)
            total += mesh.GetIndexCount(i);
        return (int)(total / 3);
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
}

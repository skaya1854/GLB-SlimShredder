using UnityEngine;
using UnityEngine.Rendering;
using UnityEditor;
using System.Collections.Generic;
using System.IO;
using System.Linq;

/// <summary>
/// Utility for rebuilding meshes with only visible triangles.
/// Preserves submesh structure, all UV channels, and vertex attributes precisely.
/// </summary>
public static class MeshBuilder
{
    private const string MeshFolder = "Assets/Meshes";

    /// <summary>
    /// Build a new mesh containing only the triangles whose global indices
    /// are in visibleTriangleIndices. Preserves submesh structure and
    /// remaps vertices so unused ones are removed.
    /// </summary>
    public static Mesh BuildCleanMesh(Mesh original, HashSet<int> visibleTriangleIndices)
    {
        // Phase 1: Iterate per-submesh to build oldToNew vertex map
        // and collect new triangles per submesh
        var oldToNew = new Dictionary<int, int>();
        var submeshTris = new List<int>[original.subMeshCount];
        int globalTriIdx = 0;

        for (int sub = 0; sub < original.subMeshCount; sub++)
        {
            var subIndices = original.GetTriangles(sub);
            int subTriCount = subIndices.Length / 3;
            submeshTris[sub] = new List<int>();

            for (int t = 0; t < subTriCount; t++)
            {
                if (visibleTriangleIndices.Contains(globalTriIdx))
                {
                    for (int k = 0; k < 3; k++)
                    {
                        int oldIdx = subIndices[t * 3 + k];
                        if (!oldToNew.ContainsKey(oldIdx))
                            oldToNew[oldIdx] = oldToNew.Count;
                        submeshTris[sub].Add(oldToNew[oldIdx]);
                    }
                }
                globalTriIdx++;
            }
        }

        // Phase 2: Remap vertex data and assemble mesh
        return AssembleRemappedMesh(original, oldToNew, submeshTris);
    }

    /// <summary>
    /// Create a new mesh by remapping vertices from the original.
    /// Preserves: positions, normals, tangents, colors, UV channels 0-7,
    /// index format, and submesh structure.
    /// </summary>
    private static Mesh AssembleRemappedMesh(
        Mesh original, Dictionary<int, int> oldToNew,
        List<int>[] submeshTris)
    {
        int count = oldToNew.Count;
        var mesh = new Mesh { name = original.name + "_clean" };

        if (count == 0) return mesh;

        // Set index format BEFORE setting any vertex/index data
        if (count > 65535)
            mesh.indexFormat = IndexFormat.UInt32;

        // Remap core vertex attributes
        var srcVerts = original.vertices;
        var verts = new Vector3[count];
        foreach (var kvp in oldToNew)
            verts[kvp.Value] = srcVerts[kvp.Key];
        mesh.vertices = verts;

        RemapNormals(mesh, original, oldToNew, count);
        RemapTangents(mesh, original, oldToNew, count);
        RemapColors(mesh, original, oldToNew, count);
        RemapAllUVChannels(mesh, original, oldToNew, count);

        // Set submeshes (preserves material slot mapping)
        mesh.subMeshCount = submeshTris.Length;
        for (int sub = 0; sub < submeshTris.Length; sub++)
            mesh.SetTriangles(submeshTris[sub], sub);

        mesh.RecalculateBounds();
        return mesh;
    }

    private static void RemapNormals(
        Mesh mesh, Mesh original, Dictionary<int, int> oldToNew, int count)
    {
        var src = original.normals;
        if (src.Length != original.vertexCount) return;

        var dst = new Vector3[count];
        foreach (var kvp in oldToNew)
            dst[kvp.Value] = src[kvp.Key];
        mesh.normals = dst;
    }

    private static void RemapTangents(
        Mesh mesh, Mesh original, Dictionary<int, int> oldToNew, int count)
    {
        var src = original.tangents;
        if (src.Length != original.vertexCount) return;

        var dst = new Vector4[count];
        foreach (var kvp in oldToNew)
            dst[kvp.Value] = src[kvp.Key];
        mesh.tangents = dst;
    }

    private static void RemapColors(
        Mesh mesh, Mesh original, Dictionary<int, int> oldToNew, int count)
    {
        var src = original.colors;
        if (src.Length != original.vertexCount) return;

        var dst = new Color[count];
        foreach (var kvp in oldToNew)
            dst[kvp.Value] = src[kvp.Key];
        mesh.colors = dst;
    }

    /// <summary>
    /// Remap UV channels 0-7 using GetUVs/SetUVs with Vector4 for full precision.
    /// </summary>
    private static void RemapAllUVChannels(
        Mesh mesh, Mesh original, Dictionary<int, int> oldToNew, int count)
    {
        for (int ch = 0; ch < 8; ch++)
        {
            var srcUVs = new List<Vector4>();
            original.GetUVs(ch, srcUVs);
            if (srcUVs.Count != original.vertexCount) continue;

            var dstUVs = new List<Vector4>(count);
            for (int i = 0; i < count; i++)
                dstUVs.Add(Vector4.zero);

            foreach (var kvp in oldToNew)
                dstUVs[kvp.Value] = srcUVs[kvp.Key];

            mesh.SetUVs(ch, dstUVs);
        }
    }

    /// <summary>
    /// Save a mesh as a .asset file under Assets/Meshes/. Returns the asset path.
    /// </summary>
    public static string SaveMeshAsset(Mesh mesh, string objectName)
    {
        EnsureFolderExists(MeshFolder);

        string baseName = SanitizeFileName(objectName) + "_clean";
        string path = $"{MeshFolder}/{baseName}.asset";
        path = GetUniquePath(path);

        AssetDatabase.CreateAsset(mesh, path);
        AssetDatabase.SaveAssets();
        return path;
    }

    private static void EnsureFolderExists(string folderPath)
    {
        if (AssetDatabase.IsValidFolder(folderPath))
            return;

        string parent = Path.GetDirectoryName(folderPath).Replace("\\", "/");
        string folderName = Path.GetFileName(folderPath);

        if (!AssetDatabase.IsValidFolder(parent))
            EnsureFolderExists(parent);

        AssetDatabase.CreateFolder(parent, folderName);
    }

    private static string GetUniquePath(string path)
    {
        if (!File.Exists(path))
            return path;

        string dir = Path.GetDirectoryName(path).Replace("\\", "/");
        string nameNoExt = Path.GetFileNameWithoutExtension(path);
        string ext = Path.GetExtension(path);

        int suffix = 1;
        string candidate;
        do
        {
            candidate = $"{dir}/{nameNoExt}_{suffix}{ext}";
            suffix++;
        }
        while (File.Exists(candidate));

        return candidate;
    }

    private static string SanitizeFileName(string name)
    {
        char[] invalid = Path.GetInvalidFileNameChars();
        return new string(name.Where(c => !invalid.Contains(c)).ToArray());
    }
}

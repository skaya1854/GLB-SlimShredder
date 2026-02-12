using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.IO;
using System.Linq;

/// <summary>
/// Utility for rebuilding meshes with only visible triangles and saving them as assets.
/// </summary>
public static class MeshBuilder
{
    private const string MeshFolder = "Assets/Meshes";

    /// <summary>
    /// Build a new mesh containing only the triangles whose indices are in visibleTriangleIndices.
    /// Remaps vertices so unused ones are removed.
    /// </summary>
    public static Mesh BuildCleanMesh(Mesh original, HashSet<int> visibleTriangleIndices)
    {
        var srcTriangles = original.triangles;

        // Collect vertex indices referenced by visible triangles
        var oldToNew = new Dictionary<int, int>();
        var newTriangles = new List<int>();

        for (int t = 0; t < srcTriangles.Length / 3; t++)
        {
            if (!visibleTriangleIndices.Contains(t)) continue;

            for (int k = 0; k < 3; k++)
            {
                int oldIdx = srcTriangles[t * 3 + k];
                if (!oldToNew.ContainsKey(oldIdx))
                    oldToNew[oldIdx] = oldToNew.Count;
                newTriangles.Add(oldToNew[oldIdx]);
            }
        }

        return AssembleRemappedMesh(original, oldToNew, newTriangles);
    }

    /// <summary>
    /// Create a new mesh by remapping vertices from the original using the old-to-new index mapping.
    /// </summary>
    private static Mesh AssembleRemappedMesh(
        Mesh original, Dictionary<int, int> oldToNew, List<int> newTriangles)
    {
        int count = oldToNew.Count;
        var srcVerts = original.vertices;
        var srcNormals = original.normals;
        var srcTangents = original.tangents;
        var srcUv = original.uv;
        var srcUv2 = original.uv2;
        var srcColors = original.colors;

        var verts = new Vector3[count];
        var normals = srcNormals.Length > 0 ? new Vector3[count] : null;
        var tangents = srcTangents.Length > 0 ? new Vector4[count] : null;
        var uv = srcUv.Length > 0 ? new Vector2[count] : null;
        var uv2 = srcUv2.Length > 0 ? new Vector2[count] : null;
        var colors = srcColors.Length > 0 ? new Color[count] : null;

        foreach (var kvp in oldToNew)
        {
            int o = kvp.Key, n = kvp.Value;
            verts[n] = srcVerts[o];
            if (normals != null) normals[n] = srcNormals[o];
            if (tangents != null) tangents[n] = srcTangents[o];
            if (uv != null) uv[n] = srcUv[o];
            if (uv2 != null) uv2[n] = srcUv2[o];
            if (colors != null) colors[n] = srcColors[o];
        }

        var mesh = new Mesh { name = original.name + "_clean" };
        mesh.vertices = verts;
        if (normals != null) mesh.normals = normals;
        if (tangents != null) mesh.tangents = tangents;
        if (uv != null) mesh.uv = uv;
        if (uv2 != null) mesh.uv2 = uv2;
        if (colors != null) mesh.colors = colors;
        mesh.triangles = newTriangles.ToArray();
        mesh.RecalculateBounds();
        return mesh;
    }

    /// <summary>
    /// Save a mesh as a .asset file under Assets/Meshes/. Returns the asset path.
    /// Handles duplicate names by appending _1, _2, etc.
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

    /// <summary>
    /// Ensure the target folder exists, creating it if necessary.
    /// </summary>
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

    /// <summary>
    /// Append numeric suffix until the path is unique.
    /// </summary>
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

    /// <summary>
    /// Remove characters that are invalid in file names.
    /// </summary>
    private static string SanitizeFileName(string name)
    {
        char[] invalid = Path.GetInvalidFileNameChars();
        return new string(name.Where(c => !invalid.Contains(c)).ToArray());
    }
}

using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// Editor window for analyzing and removing occluded (hidden) mesh faces.
/// Uses sphere raycasting + normal raycasting to detect visibility.
/// </summary>
public class OccludedFaceRemover : EditorWindow
{
    private int _sphereSamples = 1024;
    private int _raysPerSample = 8;
    private Dictionary<MeshFilter, HashSet<int>> _analysisResult;
    private Vector2 _scrollPos;

    [MenuItem("KAIST/Remove Occluded Faces")]
    private static void ShowWindow()
    {
        var window = GetWindow<OccludedFaceRemover>("Occluded Face Remover");
        window.minSize = new Vector2(350, 400);
        window.Show();
    }

    [MenuItem("KAIST/Remove Occluded Faces", true)]
    private static bool ValidateShowWindow()
    {
        return GetSelectedMeshFilters().Length > 0;
    }

    private void OnGUI()
    {
        _scrollPos = EditorGUILayout.BeginScrollView(_scrollPos);

        DrawSelectionInfo();
        EditorGUILayout.Space(10);
        DrawParameterSection();
        EditorGUILayout.Space(10);
        DrawAnalyzeButton();
        EditorGUILayout.Space(10);
        DrawAnalysisResult();
        EditorGUILayout.Space(10);
        DrawExecuteButton();

        EditorGUILayout.EndScrollView();
    }

    private void OnSelectionChange()
    {
        _analysisResult = null;
        Repaint();
    }

    /// <summary>
    /// Draw info about currently selected objects and their mesh stats.
    /// </summary>
    private void DrawSelectionInfo()
    {
        EditorGUILayout.LabelField("Selection", EditorStyles.boldLabel);

        var meshFilters = GetSelectedMeshFilters();
        if (meshFilters.Length == 0)
        {
            EditorGUILayout.HelpBox("Select GameObjects with MeshFilter components.", MessageType.Info);
            return;
        }

        int totalTriangles = meshFilters
            .Where(mf => mf.sharedMesh != null)
            .Sum(mf => GetTriangleCount(mf.sharedMesh));

        string objectNames = string.Join(", ",
            meshFilters.Select(mf => mf.gameObject.name).Distinct().Take(5));
        if (meshFilters.Select(mf => mf.gameObject.name).Distinct().Count() > 5)
            objectNames += "...";

        EditorGUILayout.LabelField("Objects", objectNames);
        EditorGUILayout.LabelField("Mesh Count", meshFilters.Length.ToString());
        EditorGUILayout.LabelField("Total Triangles", totalTriangles.ToString("N0"));
    }

    /// <summary>
    /// Draw parameter sliders for sphere samples and rays per sample.
    /// </summary>
    private void DrawParameterSection()
    {
        EditorGUILayout.LabelField("Parameters", EditorStyles.boldLabel);
        _sphereSamples = EditorGUILayout.IntSlider("Sphere Samples", _sphereSamples, 512, 4096);
        _raysPerSample = EditorGUILayout.IntSlider("Rays Per Sample", _raysPerSample, 4, 32);
    }

    /// <summary>
    /// Draw the Analyze button and handle click.
    /// </summary>
    private void DrawAnalyzeButton()
    {
        var meshFilters = GetSelectedMeshFilters();
        EditorGUI.BeginDisabledGroup(meshFilters.Length == 0);

        if (GUILayout.Button("Analyze", GUILayout.Height(30)))
        {
            _analysisResult = MeshOcclusionSolver.SolveVisibility(
                meshFilters, _sphereSamples, _raysPerSample);
        }

        EditorGUI.EndDisabledGroup();
    }

    /// <summary>
    /// Draw analysis results: total, visible, removed triangle counts and reduction percentage.
    /// </summary>
    private void DrawAnalysisResult()
    {
        if (_analysisResult == null) return;

        EditorGUILayout.LabelField("Analysis Result", EditorStyles.boldLabel);

        int totalTris = 0;
        int visibleTris = 0;

        foreach (var kvp in _analysisResult)
        {
            if (kvp.Key == null || kvp.Key.sharedMesh == null) continue;
            int meshTriCount = GetTriangleCount(kvp.Key.sharedMesh);
            totalTris += meshTriCount;
            visibleTris += kvp.Value.Count;
        }

        int removedTris = totalTris - visibleTris;
        float reductionPct = totalTris > 0 ? (float)removedTris / totalTris * 100f : 0f;

        EditorGUILayout.LabelField("Total Triangles", totalTris.ToString("N0"));
        EditorGUILayout.LabelField("Visible Triangles", visibleTris.ToString("N0"));
        EditorGUILayout.LabelField("Removed Triangles", removedTris.ToString("N0"));
        EditorGUILayout.LabelField("Reduction", $"{reductionPct:F1}%");

        // Per-mesh breakdown
        EditorGUILayout.Space(5);
        EditorGUILayout.LabelField("Per-Mesh Breakdown", EditorStyles.miniLabel);

        foreach (var kvp in _analysisResult)
        {
            if (kvp.Key == null || kvp.Key.sharedMesh == null) continue;
            int meshTotal = GetTriangleCount(kvp.Key.sharedMesh);
            int meshVisible = kvp.Value.Count;
            float meshPct = meshTotal > 0 ? (float)(meshTotal - meshVisible) / meshTotal * 100f : 0f;

            EditorGUILayout.LabelField(
                $"  {kvp.Key.gameObject.name}",
                $"{meshVisible}/{meshTotal} ({meshPct:F1}% removed)");
        }
    }

    /// <summary>
    /// Draw the Execute button (enabled only when analysis results exist).
    /// </summary>
    private void DrawExecuteButton()
    {
        if (_analysisResult == null) return;

        int visibleTris = _analysisResult.Values.Sum(s => s.Count);
        if (visibleTris == 0)
        {
            EditorGUILayout.HelpBox(
                "No visible triangles detected. Adjust parameters and re-analyze.",
                MessageType.Warning);
            return;
        }

        if (GUILayout.Button("Execute Removal", GUILayout.Height(30)))
        {
            ExecuteRemoval();
        }
    }

    /// <summary>
    /// Replace each MeshFilter's mesh with a cleaned version containing only visible triangles.
    /// Wrapped in Undo group for full undo support.
    /// </summary>
    private void ExecuteRemoval()
    {
        Undo.IncrementCurrentGroup();
        int undoGroup = Undo.GetCurrentGroup();
        Undo.SetCurrentGroupName("Remove Occluded Faces");

        int processedCount = 0;
        int totalRemoved = 0;

        foreach (var kvp in _analysisResult)
        {
            var meshFilter = kvp.Key;
            var visibleSet = kvp.Value;

            if (meshFilter == null || meshFilter.sharedMesh == null) continue;

            int originalCount = GetTriangleCount(meshFilter.sharedMesh);
            if (visibleSet.Count == originalCount) continue; // Nothing to remove

            Undo.RecordObject(meshFilter, "Remove Occluded Faces");

            var cleanMesh = MeshBuilder.BuildCleanMesh(meshFilter.sharedMesh, visibleSet);
            string assetPath = MeshBuilder.SaveMeshAsset(cleanMesh, meshFilter.gameObject.name);

            // Load the saved asset to use a persistent reference
            var savedMesh = AssetDatabase.LoadAssetAtPath<Mesh>(assetPath);
            meshFilter.sharedMesh = savedMesh;

            totalRemoved += originalCount - visibleSet.Count;
            processedCount++;
        }

        Undo.CollapseUndoOperations(undoGroup);

        EditorUtility.DisplayDialog(
            "Occluded Face Removal Complete",
            $"Processed: {processedCount} mesh(es)\nTriangles removed: {totalRemoved:N0}",
            "OK");

        _analysisResult = null;
    }

    /// <summary>
    /// Get total triangle count across all submeshes without allocating the triangles array.
    /// Uses GetIndexCount per submesh instead of mesh.triangles.Length which copies the full array.
    /// </summary>
    private static int GetTriangleCount(Mesh mesh)
    {
        uint totalIndices = 0;
        for (int i = 0; i < mesh.subMeshCount; i++)
            totalIndices += mesh.GetIndexCount(i);
        return (int)(totalIndices / 3);
    }

    /// <summary>
    /// Get all MeshFilter components from the current selection (including children).
    /// </summary>
    private static MeshFilter[] GetSelectedMeshFilters()
    {
        if (Selection.gameObjects == null || Selection.gameObjects.Length == 0)
            return new MeshFilter[0];

        return Selection.gameObjects
            .SelectMany(go => go.GetComponentsInChildren<MeshFilter>())
            .Where(mf => mf.sharedMesh != null)
            .Distinct()
            .ToArray();
    }
}

using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace SlimShredder
{
    /// <summary>
    /// Runtime MonoBehaviour for processing occluded face removal.
    /// Attach to any GameObject and call Process() with target objects.
    ///
    /// Usage:
    ///   var processor = gameObject.AddComponent&lt;OcclusionProcessor&gt;();
    ///   processor.Process(targetGameObject);
    ///   // or with callback:
    ///   processor.Process(targetGameObject, result => Debug.Log("Done"));
    /// </summary>
    public class OcclusionProcessor : MonoBehaviour
    {
        [Header("Settings")]
        [SerializeField, Range(0, 3)]
        private int _adjacencyDepth = 1;

        /// <summary>
        /// Fired during processing with (progress 0-1, description).
        /// </summary>
        public event Action<float, string> OnProgress;

        /// <summary>
        /// Result of the last Process() call.
        /// </summary>
        public OcclusionResult LastResult { get; private set; }

        /// <summary>
        /// Process all MeshFilters under the target GameObject.
        /// Synchronous — may freeze for large meshes.
        /// </summary>
        public OcclusionResult Process(GameObject target)
        {
            var meshFilters = target.GetComponentsInChildren<MeshFilter>();
            return ProcessMeshFilters(meshFilters);
        }

        /// <summary>
        /// Process specific MeshFilters.
        /// </summary>
        public OcclusionResult ProcessMeshFilters(MeshFilter[] meshFilters)
        {
            if (meshFilters == null || meshFilters.Length == 0)
                return OcclusionResult.Empty;

            // Analyze visibility
            var visibilityMap = MeshOcclusionSolver.SolveVisibility(
                meshFilters, _adjacencyDepth, true, 1, ReportProgress);

            // Apply results
            var result = ApplyResults(meshFilters, visibilityMap);
            LastResult = result;
            return result;
        }

        /// <summary>
        /// Coroutine version — yields between phases to avoid frame spikes.
        /// </summary>
        public IEnumerator ProcessAsync(GameObject target, Action<OcclusionResult> onComplete = null)
        {
            var meshFilters = target.GetComponentsInChildren<MeshFilter>();
            if (meshFilters.Length == 0)
            {
                onComplete?.Invoke(OcclusionResult.Empty);
                yield break;
            }

            // Phase 0: GPU (synchronous — fast)
            var result = new Dictionary<MeshFilter, HashSet<int>>();
            foreach (var mf in meshFilters)
                result[mf] = new HashSet<int>();

            GpuVisibilitySolver.SolveVisibility(meshFilters, 128, 1024, result, 1, ReportProgress);
            yield return null;

            // Phase 1+2: Raycast + Adjacency (synchronous)
            var fullResult = MeshOcclusionSolver.SolveVisibility(
                meshFilters, _adjacencyDepth, true, 1, ReportProgress);
            yield return null;

            // Merge GPU results into full result
            foreach (var kvp in result)
            {
                if (fullResult.ContainsKey(kvp.Key))
                {
                    foreach (int tri in kvp.Value)
                        fullResult[kvp.Key].Add(tri);
                }
            }

            // Apply
            var occlusionResult = ApplyResults(meshFilters, fullResult);
            LastResult = occlusionResult;
            onComplete?.Invoke(occlusionResult);
        }

        // ────────────────────────────────────────────────────────────────

        private OcclusionResult ApplyResults(
            MeshFilter[] meshFilters,
            Dictionary<MeshFilter, HashSet<int>> visibilityMap)
        {
            int processedCount = 0;
            int deactivatedCount = 0;
            int totalRemoved = 0;

            foreach (var mf in meshFilters)
            {
                if (mf == null || mf.sharedMesh == null) continue;
                if (!visibilityMap.TryGetValue(mf, out var visibleSet)) continue;

                int originalCount = GetTriangleCount(mf.sharedMesh);
                if (visibleSet.Count == originalCount) continue;

                // Fully occluded — deactivate
                if (visibleSet.Count == 0)
                {
                    mf.gameObject.SetActive(false);
                    totalRemoved += originalCount;
                    deactivatedCount++;
                    continue;
                }

                // Partially visible — rebuild mesh
                var cleanMesh = MeshBuilder.BuildCleanMesh(mf.sharedMesh, visibleSet);
                mf.sharedMesh = cleanMesh;

                totalRemoved += originalCount - visibleSet.Count;
                processedCount++;
            }

            var occlusionResult = new OcclusionResult(
                processedCount, deactivatedCount, totalRemoved);

            Debug.Log($"[OcclusionProcessor] Done: {processedCount} rebuilt, "
                    + $"{deactivatedCount} deactivated, "
                    + $"{totalRemoved:N0} triangles removed");

            return occlusionResult;
        }

        private void ReportProgress(float progress, string description)
        {
            OnProgress?.Invoke(progress, description);
        }

        private static int GetTriangleCount(Mesh mesh)
        {
            uint totalIndices = 0;
            for (int i = 0; i < mesh.subMeshCount; i++)
                totalIndices += mesh.GetIndexCount(i);
            return (int)(totalIndices / 3);
        }
    }

    /// <summary>
    /// Immutable result of an occlusion processing operation.
    /// </summary>
    public readonly struct OcclusionResult
    {
        public readonly int ProcessedMeshes;
        public readonly int DeactivatedObjects;
        public readonly int TrianglesRemoved;

        public OcclusionResult(int processed, int deactivated, int removed)
        {
            ProcessedMeshes = processed;
            DeactivatedObjects = deactivated;
            TrianglesRemoved = removed;
        }

        public static OcclusionResult Empty => new OcclusionResult(0, 0, 0);
    }
}

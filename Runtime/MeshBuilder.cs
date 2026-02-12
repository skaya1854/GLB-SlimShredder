using UnityEngine;
using UnityEngine.Rendering;
using System.Collections.Generic;
using System.Linq;

namespace SlimShredder
{
    /// <summary>
    /// Runtime utility for rebuilding meshes with only visible triangles.
    /// Preserves submesh structure, all UV channels, and vertex attributes.
    /// </summary>
    public static class MeshBuilder
    {
        /// <summary>
        /// Build a new mesh containing only the triangles whose global indices
        /// are in visibleTriangleIndices. Preserves submesh structure and
        /// remaps vertices so unused ones are removed.
        /// </summary>
        public static Mesh BuildCleanMesh(Mesh original, HashSet<int> visibleTriangleIndices)
        {
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

            return AssembleRemappedMesh(original, oldToNew, submeshTris);
        }

        private static Mesh AssembleRemappedMesh(
            Mesh original, Dictionary<int, int> oldToNew,
            List<int>[] submeshTris)
        {
            int count = oldToNew.Count;
            var mesh = new Mesh { name = original.name + "_clean" };

            if (count == 0)
            {
                mesh.vertices = new Vector3[0];
                return mesh;
            }

            if (count > 65535)
                mesh.indexFormat = IndexFormat.UInt32;

            var srcVerts = original.vertices;
            var verts = new Vector3[count];
            foreach (var kvp in oldToNew)
                verts[kvp.Value] = srcVerts[kvp.Key];
            mesh.vertices = verts;

            RemapNormals(mesh, original, oldToNew, count);
            RemapTangents(mesh, original, oldToNew, count);
            RemapColors(mesh, original, oldToNew, count);
            RemapAllUVChannels(mesh, original, oldToNew, count);

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
    }
}

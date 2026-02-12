# GLB-SlimShredder

Unity tool that removes occluded (hidden) mesh faces from 3D models.
Combines **GPU rendering**, **multi-hit raycasting**, and **adjacency expansion** to accurately detect visible geometry — then rebuilds meshes with only the surviving triangles.

Supports both **Editor** (EditorWindow UI) and **Runtime** (MonoBehaviour API).

## Features

- **3-Phase Visibility Detection**
  - Phase 0: GPU Triangle ID rendering from 128 viewpoints (CommandBuffer)
  - Phase 1: Multi-hit sphere raycasting (4 penetration hits per ray)
  - Phase 2: N-ring adjacency expansion (boundary hole prevention)
- **Auto Deactivation**: Fully occluded objects (0 visible faces) are automatically deactivated
- **Editor Mode**: EditorWindow with analyze/preview, undo support, asset export
- **Runtime Mode**: MonoBehaviour with sync/coroutine API, progress callback

## Installation

```
Editor only:  Copy Editor/ → Assets/Scripts/Editor/
Runtime:      Copy Runtime/ → Assets/Scripts/ (any non-Editor folder)
Both:         Copy both folders
```

---

## Editor Usage

1. Select GameObjects with MeshFilter components
2. Open **Tools > Remove Occluded Faces**
3. Adjust parameters:
   - **Sphere Samples** (512–8192): Raycast viewpoints on bounding sphere
   - **Rays Per Sample** (4–32): Jittered rays per viewpoint
   - **Adjacency Depth** (0–3): N-ring neighbor preservation depth
4. Click **Analyze** to preview results
5. Click **Execute Removal** to apply (with Undo support)

---

## Runtime Usage

All runtime classes are in the `SlimShredder` namespace.

### Quick Start (Synchronous)

```csharp
using SlimShredder;

var processor = gameObject.AddComponent<OcclusionProcessor>();
OcclusionResult result = processor.Process(targetGameObject);

Debug.Log($"Removed {result.TrianglesRemoved} triangles, "
        + $"deactivated {result.DeactivatedObjects} objects");
```

### Coroutine (Non-blocking)

```csharp
using SlimShredder;

var processor = gameObject.AddComponent<OcclusionProcessor>();
processor.OnProgress += (progress, desc) => loadingBar.value = progress;

StartCoroutine(processor.ProcessAsync(targetGameObject, result =>
{
    Debug.Log($"Done: {result.TrianglesRemoved} triangles removed");
}));
```

### Direct API (Advanced)

```csharp
using SlimShredder;

MeshFilter[] meshes = target.GetComponentsInChildren<MeshFilter>();

// Step 1: Analyze
var visibilityMap = MeshOcclusionSolver.SolveVisibility(
    meshes, sphereSamples: 2048, raysPerSample: 16, adjacencyDepth: 1);

// Step 2: Rebuild individually
foreach (var (mf, visibleSet) in visibilityMap)
{
    if (visibleSet.Count == 0) { mf.gameObject.SetActive(false); continue; }
    mf.sharedMesh = MeshBuilder.BuildCleanMesh(mf.sharedMesh, visibleSet);
}
```

### Runtime Setup

Add `Hidden/Internal-Colored` to **Project Settings > Graphics > Always Included Shaders** for GPU visibility to work in builds.

---

## Algorithm

### Phase 0 — GPU Rendering (most accurate)

Renders all meshes from 128 Fibonacci-distributed viewpoints using a `CommandBuffer`.
Each triangle is assigned a unique ID encoded as vertex color (24-bit RGB).
Reads back pixels from a float RenderTexture to decode which triangles are visible.

### Phase 1 — Multi-hit Sphere Raycasting (supplementary)

Generates uniformly distributed points on a bounding sphere (Fibonacci spiral).
From each point, casts rays toward the mesh center with jittered spread.
Each ray penetrates up to **4 surfaces** (`RaycastCommand` with `maxHits=4`), marking all hit triangles as visible.

### Phase 2 — Adjacency Expansion (safety net)

Builds an edge-to-triangle adjacency map.
Expands the visible set by N rings — marking neighbors of visible triangles as visible too.
Prevents holes at the boundary between visible and removed geometry.

### Execution

- Partially visible meshes → rebuilt with only visible triangles
- Fully occluded meshes (0 visible) → `GameObject.SetActive(false)`

## Files

| Folder | File | Description |
|--------|------|-------------|
| `Editor/` | `GpuVisibilitySolver.cs` | GPU Triangle ID rendering (Editor) |
| `Editor/` | `MeshOcclusionSolver.cs` | 3-phase orchestrator (Editor) |
| `Editor/` | `MeshBuilder.cs` | Mesh reconstruction + asset saving (Editor) |
| `Editor/` | `OccludedFaceRemover.cs` | EditorWindow UI |
| `Runtime/` | `GpuVisibilitySolver.cs` | GPU Triangle ID rendering (Runtime) |
| `Runtime/` | `MeshOcclusionSolver.cs` | 3-phase orchestrator (Runtime) |
| `Runtime/` | `MeshBuilder.cs` | Mesh reconstruction (Runtime) |
| `Runtime/` | `OcclusionProcessor.cs` | MonoBehaviour wrapper + OcclusionResult |

## Requirements

- Unity 6+ (URP compatible)
- Runtime: `Hidden/Internal-Colored` in Always Included Shaders

## Notes

- Submesh structure and material slots are preserved
- All vertex attributes preserved (positions, normals, tangents, colors, UV0–UV7)
- Uses Physics layer 31 temporarily during raycast analysis
- GPU phase uses float RT to avoid sRGB precision loss

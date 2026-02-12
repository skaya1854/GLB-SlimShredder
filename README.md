# GLB-SlimShredder

Unity Editor tool that removes occluded (hidden) mesh faces from 3D models.
Combines **GPU rendering**, **multi-hit raycasting**, and **adjacency expansion** to accurately detect visible geometry — then rebuilds meshes with only the surviving triangles.

## Features

- **3-Phase Visibility Detection**
  - Phase 0: GPU Triangle ID rendering from 128 viewpoints (CommandBuffer)
  - Phase 1: Multi-hit sphere raycasting (4 penetration hits per ray)
  - Phase 2: N-ring adjacency expansion (boundary hole prevention)
- **Auto Deactivation**: Fully occluded objects (0 visible faces) are automatically deactivated
- **EditorWindow UI**: Configurable parameters with analyze/preview before execution
- **Undo Support**: Full undo/redo via Unity's Undo system
- **Asset Export**: Saves cleaned meshes as `.asset` files — originals remain untouched

## Installation

Copy the `Editor/` folder into your Unity project's `Assets/Scripts/Editor/` (or any `Editor/` folder).

## Usage

1. Select GameObjects with MeshFilter components in the Hierarchy
2. Open **Tools > Remove Occluded Faces** from the menu bar
3. Adjust parameters:
   - **Sphere Samples** (512–8192): Raycast viewpoints on bounding sphere
   - **Rays Per Sample** (4–32): Jittered rays per viewpoint
   - **Adjacency Depth** (0–3): N-ring neighbor preservation depth
4. Click **Analyze** to preview results
5. Review per-mesh breakdown (total / visible / removed triangles)
6. Click **Execute Removal** to apply

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

- Partially visible meshes → rebuilt with only visible triangles, saved as new `.asset`
- Fully occluded meshes (0 visible) → `GameObject.SetActive(false)` (prevents empty mesh export errors)

## Files

| File | Description |
|------|-------------|
| `Editor/GpuVisibilitySolver.cs` | GPU-based Triangle ID rendering with CommandBuffer |
| `Editor/MeshOcclusionSolver.cs` | 3-phase orchestrator (GPU + Raycast + Adjacency) |
| `Editor/MeshBuilder.cs` | Mesh reconstruction, vertex remapping, and asset saving |
| `Editor/OccludedFaceRemover.cs` | EditorWindow UI and execution logic |

## Requirements

- Unity 6+ (URP compatible)
- Editor only (no runtime dependency)

## Notes

- Submesh structure and material slots are preserved on cleaned meshes
- All vertex attributes preserved (positions, normals, tangents, colors, UV0–UV7)
- Uses Physics layer 31 temporarily during raycast analysis
- GPU phase uses `Hidden/Internal-Colored` shader with float RT to avoid sRGB precision loss

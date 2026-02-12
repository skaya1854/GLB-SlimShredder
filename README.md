# GLB-SlimShredder

Unity Editor tool that removes occluded (hidden) mesh faces from 3D models, keeping only externally visible geometry.

## Features

- **Sphere Raycasting** (Primary): Casts rays from evenly distributed points on a bounding sphere to detect visible faces
- **Normal Raycasting** (Secondary): Validates remaining faces by casting rays along triangle normals
- **EditorWindow UI**: Configurable parameters with analyze/preview before execution
- **Undo Support**: Full undo/redo via Unity's Undo system
- **Asset Export**: Saves cleaned meshes as `.asset` files preserving original models

## Installation

Copy the `Editor/` folder into your Unity project's `Assets/Scripts/Editor/` (or any `Editor/` folder).

## Usage

1. Select GameObjects with MeshFilter components in the Hierarchy
2. Open `KAIST > Remove Occluded Faces` from the menu bar
3. Adjust parameters:
   - **Sphere Samples** (512-4096): Number of viewpoints on bounding sphere
   - **Rays Per Sample** (4-32): Additional jittered rays per viewpoint
4. Click **Analyze** to preview results
5. Review per-mesh breakdown (total/visible/removed triangles)
6. Click **Execute Removal** to apply

## Algorithm

### Phase 1: Sphere Raycasting (0-70%)
Generates uniformly distributed points on a bounding sphere (Fibonacci spiral).
From each point, casts rays toward the mesh center with jittered spread.
First-hit triangles are marked as visible.

### Phase 2: Normal Raycasting (70-100%)
For unmarked triangles, casts a ray from the triangle center along its outward normal.
If the ray escapes without hitting other geometry, the face is externally visible.

## Files

| File | Description |
|------|-------------|
| `Editor/MeshOcclusionSolver.cs` | Core visibility algorithm (Fibonacci sphere + raycasting) |
| `Editor/MeshBuilder.cs` | Mesh reconstruction and asset saving |
| `Editor/OccludedFaceRemover.cs` | EditorWindow UI and orchestration |

## Requirements

- Unity 6+ (URP compatible)
- Editor only (no runtime dependency)

## Notes

- Textures/materials are not preserved on cleaned meshes (geometry only)
- Original meshes remain untouched; cleaned meshes are saved as new assets
- Uses Physics layer 31 temporarily during analysis

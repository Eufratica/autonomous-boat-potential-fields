# 3D Modelling in xasv-sim

xasv-sim uses a **Blender-first pipeline** for both river environments and the ASV robot model.
This file gives a detailed overview of how Blender scenes are converted into Gazebo-ready SDF/URDF models.

---

## 1. Environment Modelling (Worlds)

### 1.1 Authoring in Blender

River environments (e.g., the hydroelectric plant scenario) are first created in Blender using standard meshes, materials, and textures. Typical elements include:

- riverbed and water surface
- dam structures and piers
- buoys and markers
- vegetation and surrounding terrain

All geometry, materials, and layout are authored directly in Blender.

### 1.2 Export via `sdf_exporter.py`

Once the scene is ready, the user runs a Blender Python script (`sdf_exporter.py`) from inside Blender. Using the `bpy` API, the script:

1. Iterates over all (visible) mesh objects in the scene.
2. Exports **one COLLADA (`.dae`) file per object**.
3. Automatically builds:
   - a `model_raw.sdf` (with one link per exported object);
   - a final `model.sdf` with **simplified collision geometry**;
   - the corresponding `model.config`;
   - and a `meshes/` directory with all exported meshes.

By default, the output is written under:

```text
~/.gazebo/models/<MODEL_NAME>/
```

but both the base directory and the model name can be edited at the top of sdf_exporter.py (e.g., MODEL_NAME and BASE_DIR).

### 1.3 Example: SAE_HYDRO_V0

This repository includes a complete example following this workflow in:

```text
xasv_sim/models/SAE_HYDRO_V0
```

This model represents a hydroelectric plant environment and serves both as:

* a ready-to-use world for ASV navigation and inspection experiments; and

* a template that can be adapted to other riverine or inspection scenarios.

### 1.4 Blender-to-Gazebo Environment Workflow

The figure below illustrates the environment-export pipeline:

* Left: the SAE_HYDRO_V0 environment loaded in Gazebo after exporting meshes and SDF files with sdf_exporter.py.

* Right: the same scene in Blender, where geometry, materials, and layout are authored.

<p align="center">
  <img src="figs/env_exporter.png" alt="Blender-to-Gazebo environment export workflow" width="600"/>
</p>

## 2. ASV Robot Modelling (migbot2)

### 2.1 Blender Scene Structure

For the ASV robot, we follow a similar Blender-first approach, but the final output is a URDF xacro instead of an SDF world.

* In Blender, we keep a single assembled scene containing:

* the hull and superstructure;

* collision boxes named col0, col1, …, colN;

* propellers named helice*;

* and a visible base_link object.

This makes it easier to visualize and edit the robot as a whole while preserving a clear naming convention for export.

### 2.2 Export via migbot2_xacro_exporter.py

A dedicated bpy script (migbot2_xacro_exporter.py) installs a custom export entry in Blender:

```text
File → Export → Export Xacro (migbot2_ardupilot)
```

When invoked, the script:

* Duplicates each visible mesh to a temporary object.

* Bakes rotation and scale into the geometry (keeping the location unchanged).

* Triangulates the mesh and applies smooth shading with weighted normals.

* Exports a set of COLLADA meshes (.dae) into a meshes/ folder.

* Generates a structured migbot2.urdf.xacro alongside the meshes.

### 2.3 Material and Texture Post-Processing

During export, the script post-processes the DAE files so that materials appear in Gazebo close to the Blender viewport:

* Rewrites COLLADA effects to use a PHONG shader.

* Converts flat diffuse colors into small 1×1 PNG textures stored next to each mesh.

* Retargets image references so that OGRE’s material system in Gazebo can load them robustly.

### 2.4 Collision and Inertia Generation

At the same time, the script builds a structured URDF/xacro robot:

* base_link receives:

  * a collision model computed from the col* cubes (simple boxes); and

  * an approximate inertia based on a configurable density parameter.

* Each propeller becomes a Helice_i link with:

  * a cylindrical collision shape; and

  * a continuous joint.

* Remaining meshes get:

  * simple box collision geometries; and

  * inertias derived from their axis-aligned bounding box.

  * This keeps the model lightweight and suitable for real-time simulation

### 2.5 Injected Gazebo Plugins via xacro

The generated migbot2.urdf.xacro injects a reusable xacro block with the Gazebo plugins used in xasv-sim, including:

* six gazebo_motor_model thruster instances;

* USV dynamics and buoyancy plugins;

* a follow plugin;

* IMU, magnetometer, and GPS sensor plugins;

* optional gazebo_ros_control transmissions.

These plugins are parameterized via xacro arguments and properties (e.g., ${namespace}, ${ardupilot}, reference latitude/longitude, fluid density, hull dimensions). The same exported migbot2.urdf.xacro can be launched either:

* in pure ROS control mode, or

* in ArduPilot SITL/HITL mode,

* by toggling parameters in the launch files.

### 2.6 Blender-to-Gazebo Robot Workflow

The figure below illustrates the robot-export pipeline:


<p align="center">
  <img src="figs/bot_exporter.png" alt="Blender-to-Gazebo robot export workflow" width="600"/>
</p>

* Left: the ASV model loaded in Gazebo with dynamics and sensor plugins;

* Right: the corresponding migbot2 model in Blender.

# Summary

* Environments: modeled in Blender and exported via sdf_exporter.py to SDF + model.config + meshes/.

* Robot (migbot2): modeled in Blender and exported via migbot2_xacro_exporter.py to migbot2.urdf.xacro + meshes/, with collisions, inertias, and Gazebo plugins integrated.

This Blender-first workflow keeps visual and physical definitions coherent and reproducible, while allowing users to iterate on geometry and appearance directly in Blender and regenerate the Gazebo models with a single export step.









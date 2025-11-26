# LASER Gazebo Resources

This package provides the essential **assets, plugins, and world files** required for the Gazebo simulation environment of the Laser UAV System (LUS). It serves as a resource library that other simulation packages (like `laser_uav_simulation`) depend on.

## Overview

The `laser_gazebo_resources` package bridges the gap between standard Gazebo capabilities and the specific needs of our UAV research. It implements custom C++ plugins to simulate realistic sensor behaviors (specifically Livox LiDARs) and actuator dynamics, alongside providing custom 3D models and environments.

### Key Features
* **High-Fidelity LiDAR Simulation:** Custom plugins to simulate the unique non-repetitive scanning patterns of Livox LiDARs (e.g., Mid-360).
* **Advanced Motor Dynamics:** Plugins to simulate motor thrust, torque, and response times more accurately than standard plugins.
* **Custom Environments:** Ready-to-use Gazebo world files for different testing scenarios.
* **Protobuf Integration:** Includes Protobuf message definitions for efficient communication with flight stacks (like PX4 SITL).

## Provided Plugins

These plugins are exported as shared libraries and can be referenced in SDF/URDF files.

### 1. `LivoxPointsPlugin`
-   **Description:** Simulates a Livox LiDAR sensor. Unlike standard ray-cast LiDARs, this plugin generates points based on a specific scanning pattern defined in a CSV file.
-   **Key Capabilities:**
    -   Raycasting using Gazebo physics.
    -   Support for CSV-based scan patterns (e.g., `mid360-real-centr.csv`).
    -   Publishes PointCloud2 messages to ROS 2.


### 2. `GazeboMotorModel`
-   **Description:** Simulates the physics of a brushless DC motor and propeller. It takes a normalized command input and applies force and torque to the rigid body link in Gazebo.
-   **Features:**
    -   Configurable motor constants and propeller dynamics.
    -   Simulates motor time constant (spin-up/spin-down delay).
    -   Interacts with `rotors_model` logic.

### 3. `LinkStaticTfPublisher`
-   **Description:** A helper plugin that publishes a static TF transform for a specific link in the simulation, useful for ground truth or sensor frames that are fixed relative to the world or parent.

## Provided Assets

### Models (`/models`)
-   **`livox_sensor`**: A generic model representing a Livox unit, used in conjunction with the plugin.
-   **`tree_simple`**: A lightweight 3D tree model used to populate the forest environment without consuming excessive GPU resources.

### Worlds (`/worlds`)
-   **`custom_empty.world`**: A basic world with a physics setup optimized for UAV flight (proper gravity, sun, and ground plane).
-   **`forest.world`**: A generated environment populated with `tree_simple` models to test obstacle avoidance and mapping algorithms in cluttered spaces.

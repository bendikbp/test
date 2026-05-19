# Third-Party Notices

This project uses third-party software, frameworks, models, textures, shaders, and other assets. This file includes the main external components used in the bachelor project and provides sources, licence information, and notes about modifications.

<br>

## Project Code
The source code developed by the bachelor project group is included for documentation, evaluation, and reproducibility of the bachelor project.

No open-source licence is applied to the project code. Unless otherwise explicitly stated, all rights to the project-specific source code are reserved by the project authors.

Third-party software, frameworks, models, textures, shaders, and proprietary material remain subject to their own licences or terms of use, as listed in this file.

<br>

## Software Frameworks and Librarires

### ROS 2
- Website: https://docs.ros.org
- License: Apache 2.0 / mixed by package
- Usage: Middleware, nodes, communication framework, topics, services, launch files, TF transformations
- Modified: No

### Gazebo Sim
- Website: https://gazebosim.org
- Source: https://github.com/gazebosim/gz-sim
- License: Apache 2.0
- Usage: Physics simulation, rendering, world simulation
- Modified: No

<br>

## 3D Models / Assets

### Havyard 842 1/50 AHTS Tug Vessel Model
- Source: https://grabcad.com/library/havyard-842-1-50-ahts-tug-1
- Author: Sigbjørn Mork
- Platoform: GrabCAD Community Library
- License/terms: GrabCAD Website Terms of Use / User Submission Cross License
- Modifications: Converted, simplified, and optimised for performance increase in Gazebo
- Notes: The original author and source are credited. The model is not treated as open-source

### VRX Ocean Wave Assets
- Source: https://github.com/osrf/vrx
- Author: Open Source Robotics Foundation
- Licence: Apache 2.0
- Usage: Visual ocean wave deformation and surface effects in Gazebo
- Modified: No

### Ocean Surface Mesh
- Source: https://github.com/srmainwaring/wave_sim_vrx/tree/master/wave_gazebo/world_models/ocean_waves/meshes
- Author: Rhys Mainwaring
- Licence: Apache 2.0
- Usage: Geometric mesh used as the visual ocean surface in the Gazebo environment
- Modified: No

### Ocean Surface Texture
- Source: https://github.com/srmainwaring/wave_sim_vrx/tree/master/wave_gazebo/world_models/ocean_waves/materials/textures
- Author: Rhys Mainwaring
- Licence: Apache 2.0
- Usage: Texture asset used by the ocean material to give the visual water surface a more realistic appearance in Gazebo
- Modified: No

<br>

## Project-Specific
### Palfinger Marine AS Models
- Source: Palfinger Marine AS
- Licence/terms: Project-specific/proprietary material
- Usage: Reference material for crane and platform
- Modified: Simplified and optimised for better Gazebo performance
- Notes: Material provided by Palfinger Marine AS is not considered open-source third-party material

<br>

## Redistribution Limitations

Confidential files and third-party material that cannot be redistributed are not included in the public repository. This includes restricted files from Palfinger Marine AS and third-party assets with redistribution limitations.

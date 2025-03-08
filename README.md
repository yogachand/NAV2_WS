## 1. my_robot_description 
This folder contains the `my_robot_description` package, which provides the robot's Unified Robot Description Format (URDF) files. These files, written in XML, define the robot's structure, physical properties, and joint configurations. Additionally, a Python launch file is included for easily visualizing the robot in RViz and simulating it in Gazebo.
**Key Features:**
-   Detailed URDF model of the rover.
-   Python launch file for seamless RViz and Gazebo integration.
-   Accurate representation of the robot's physical properties and kinematics.
-   Enables robot simulation and visualization.
**How to Use:**
1.  **Build the Workspace:** Ensure your ROS 2 workspace is built. Navigate to your workspace's root directory and run:
    ```bash
    colcon build
2.  **Source the Workspace:** Source the workspace's setup file:
    ```bash
    source install/setup.bash
    ```
3.  **Launch the Robot Description:** Launch the Python launch file to visualize the robot in RViz and Gazebo:
    ```bash
    ros2 launch my_robot_description robot.launch.py
    ```
    This command will open RViz with the robot's 3D model and, if Gazebo is configured, launch a Gazebo simulation of the robot.
**Contents:**
-   `urdf/`: Contains the XML URDF files defining the robot's structure.
-   `launch/`: Contains the Python launch file (`robot_description.launch.py`) for RViz and Gazebo.
**Notes:**
-   Ensure you have RViz and Gazebo installed for visualization and simulation.
-   The launch file may require modifications depending on your specific Gazebo setup and desired simulation environment.
-   If you have custom meshes, ensure they are correctly referenced in the URDF files and placed in the appropriate location.


## 2. Navigation and SLAM (robot_mapping)

The `robot_mapping` folder contains the configuration and launch files necessary for the rover's navigation and Simultaneous Localization and Mapping (SLAM) capabilities.

**Folder Structure:**

-   `config/`: This folder contains XML configuration files for the navigation and SLAM algorithms. These files define parameters for path planning, map building, and localization.
-   `launch/`: This folder contains Python launch files used to start the SLAM and navigation systems. These launch files manage the launch of necessary nodes and set parameters.
-   `robot_mapping`: This folder contains the Python script `mapping_with_known_poses.py`, which generates a basic map using the Bresenham algorithm to draw straight lines based on lidar data. This script serves as a foundational map generation tool prior to implementing SLAM. While successful in generating maps with the real rover, odometry issues prevented the integration of these maps into real-time rover navigation. **However, this code functions correctly within a simulation environment.**

**Key Features:**

-   Configuration files for customizing navigation and SLAM behavior.
-   Launch files for simplified startup of navigation and SLAM systems.
-   Dedicated tools for mapping with known poses.
-   Integration of SLAM and navigation algorithms for autonomous operation.

**How to Use:**

1.  **Configuration:** Modify the XML files in the `config/` folder to adjust navigation and SLAM parameters to suit your specific environment and requirements.
2.  **Launch:** Use the launch files in the `launch/` folder to start the desired navigation or SLAM system. For example:

    ```bash
    ros2 launch robot_mapping online_async.launch.py
    ros2 launch robot_mapping navigation.launch.py
3.  ros2 run robot_mapping mapping_with_known_posses.py this files helps us to draw the map.
**Notes:**

-   Ensure you have the necessary ROS 2 navigation and SLAM packages installed.
-   The specific launch files and configuration files may vary depending on the chosen SLAM algorithm and navigation stack.
-   Refer to the documentation for the specific SLAM and navigation packages used for detailed information on configuration parameters.


##3 robot_safety_feature Package
This ROS 2 package, `robot_safety_feature`, provides a robust safety system for your robot using lidar scan data. It enables configurable scanning angles, implements safety zones, and offers visual feedback through RViz markers.

## Features
* **Lidar Scan Subscription:** Subscribes to lidar scan data to detect obstacles in the robot's environment.
* **Adjustable Scanning Angles:** Allows users to configure the lidar scanning angle to focus on specific areas of interest.
* **Self-Clearance:** Implements a 10cm self-clearance radius to prevent the lidar from detecting the robot's own components.
* **Safety Zones:** Defines three distinct zones:
    * **Danger Zone:** Indicates an immediate collision risk.
    * **Warning Zone:** Provides an early warning of potential obstacles.
    * **Free Zone:** Represents a safe area for robot movement.
* **RViz Visualization:** Uses RViz markers to visually represent the defined safety zones, providing clear feedback to the user.

## Installation
1.  **Clone the Repository:** Clone this repository into your ROS 2 workspace's `src` directory.

    ```bash
    cd <your_ros2_workspace>/src
    git clone <repository_url>
    ```
2.  **Build the Package:** Build the package using `colcon`.
    ```bash
    cd <your_ros2_workspace>
    colcon build --packages-select robot_safety_feature
    ```
3.  **Source the Workspace:** Source the workspace's setup file.
    ```bash
    source install/setup.bash
    ```
## Usage
1.  **Launch the Node:** Launch the `robot_safety_feature` node.
    ```bash
    ros2 run robot_safety_feature safety_node.py 
2.  **View in RViz:** Open RViz and add the marker display to visualize the safety zones.
3.  **Configure Parameters:** Modify the parameters in the launch file or through dynamic reconfigure to adjust the scanning angles and zone thresholds.

## Configuration

The following parameters can be configured:

* **Scanning Angle:** Adjust the lidar scanning angle to focus on specific areas.
* **Danger Zone Threshold:** Define the distance threshold for the danger zone.
* **Warning Zone Threshold:** Define the distance threshold for the warning zone.


## 3. SICK_SCAN_WS (SICK Scan)

This folder contains the instructions and configuration files for the robot's safety system, utilizing a SICK lidar scanner. Due to the size of the `sick_scan_ws` workspace, it is not included directly in this repository. Instead, we provide a link to the original SICK lidar driver repository, which we used as a foundation for our implementation:

[Link to the SICK lidar driver repository for the TIM781]
https://github.com/SICKAG/sick_scan_xd


**Key Features:**

-   Obstacle detection.
-   Emergency stop functionality.
-   Safety zone monitoring.

**Implementation Details:**

We followed the instructions and configurations provided in the linked repository. We have adapted and customized the driver to integrate with our rover's system. Please refer to the linked repository for installation and basic usage instructions. Our specific adaptations are documented in the configuration files and any custom scripts within this folder.

**How to Configure and Run:**

1.  Create a ROS 2 workspace for your rover, if you don't have one already.
2.  Follow the ROS 2 installation instructions provided in the linked SICK TIM781 driver repository on GitHub.
3.  Copy the configuration files and custom scripts from this repository's `sick_scan` folder into your ROS 2 workspace, within the appropriate package.

**Important:** You must clone the original SICK lidar driver repository to use the key Features.

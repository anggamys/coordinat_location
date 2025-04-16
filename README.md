# `coordinat_location` Package

## Overview

The `coordinat_location` package provides tools for working with geographical coordinates. It includes features such as coordinate system conversion, distance calculation, and other geographic computations.

## Installation

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros_workspace/src
git clone https://github.com/anggamys/coordinat_location.git
```

Build the package using `colcon`:

```bash
cd ~/ros_workspace
colcon build --packages-select coordinat_location
```

Source the workspace and run the node:

```bash
source ~/ros_workspace/install/setup.bash
ros2 run coordinat_location rtk_node
```

## Usage

For detailed usage instructions, refer to the [`docs/usage.md`](docs/usage.md) file. It contains examples and tutorials to help you get started with the package's functionality.

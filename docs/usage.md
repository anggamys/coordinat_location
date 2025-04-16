# Usage Guide

## Available Commands

### 1. `rtk_node`

The `rtk_node` is responsible for processing RTK (Real-Time Kinematic) data and providing accurate location information.

#### Run the Node

To start the node with default parameters:

```bash
ros2 run coordinat_location rtk_node
```

#### Available Parameters

The node accepts the following parameters:

- `--caster`: NTRIP caster hostname (default: `nrtk.big.go.id`)
- `--port`: Port number for the caster (default: `2101`)
- `--mountpoint`: Mount point used to access RTK data (default: `RTCM3`)
- `--username`: Username for authenticating with the caster (default: `username`)
- `--password`: Password for authenticating with the caster (default: `password`)

#### Example with Custom Parameters

To run the node with your own NTRIP caster credentials:

```bash
ros2 run coordinat_location rtk_node --ros-args \
  -p caster:=<caster> \
  -p port:=<port> \
  -p mountpoint:=<mountpoint> \
  -p username:=<username> \
  -p password:=<password>
```

> Replace `<caster>`, `<port>`, `<mountpoint>`, `<username>`, and `<password>` with your actual NTRIP caster credentials.

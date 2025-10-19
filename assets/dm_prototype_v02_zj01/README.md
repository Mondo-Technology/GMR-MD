# DM Prototype V02 Retargeting Setup

This document describes the motion retargeting setup for the DM Prototype V02 robot.

## Files Created

### 1. XML Model
**Location**: `xml/dm_prototype_v02_mocap.xml`

This is the MuJoCo XML model configured for motion capture retargeting with:
- **Free joint** on `base_link` for 6-DOF motion (3 position + 4 quaternion rotation)
- **23 actuated joints** for the robot (waist, legs, arms)
- **Mocap bodies** for foot tracking (`l_foot_mocap`, `r_foot_mocap`)
- **Sensors** for IMU simulation

**Key features**:
- Robot starts at height 0.5m
- Visual and collision geometries for all links
- Actuator force ranges configured for each joint
- Simplified box geometries for wrist roll links (corrupted STL files)

### 2. IK Configuration
**Location**: `../../general_motion_retargeting/ik_configs/smplx_to_dm_prototype_v02.json`

Configuration for inverse kinematics retargeting from SMPL-X to DM Prototype V02:

**Scaling factors** (relative to 1.8m human):
- Torso/pelvis/legs: 0.6x scale
- Arms: 0.55x scale

**Two-stage IK matching**:
- **Stage 1** (ik_match_table1): Focuses on orientation matching (high rotation weights)
- **Stage 2** (ik_match_table2): Focuses on position matching (high position weights, especially feet)

**Key mappings**:
| Robot Body | Human Body | Priority |
|------------|------------|----------|
| base_link | pelvis | High position + orientation |
| l/r_foot_mocap | left/right_foot | Very high (position + orientation) |
| waist_yaw_link | spine3 | Orientation only |
| Leg links | leg joints | Progressive position weights |
| Arm links | arm joints | Progressive position weights |

### 3. Parameter Registration
**Location**: `../../general_motion_retargeting/params.py`

The robot has been registered in the system with:
- XML path mapping
- IK config mapping for SMPL-X source
- Base link name: `base_link`
- Camera distance: 1.5m

## Usage

### Basic Retargeting

```bash
# Retarget SMPL-X motion to DM Prototype V02
python scripts/smplx_to_robot.py \
    --smplx_file <path_to_smplx_file.pkl> \
    --robot dm_prototype_v02 \
    [--save_path <output.pkl>] \
    [--loop] \
    [--record_video] \
    [--rate_limit]
```

### With IsaacLab

```bash
# Retarget and visualize in IsaacLab
python scripts/smplx_to_robot_isaaclab.py \
    --smplx_file <path_to_smplx_file.pkl> \
    --robot dm_prototype_v02 \
    [--save_path <output.pkl>] \
    [--headless]
```

### Programmatic Usage

```python
from general_motion_retargeting import GeneralMotionRetargeting as GMR

# Initialize retargeting system
retarget = GMR(
    src_human="smplx",
    tgt_robot="dm_prototype_v02",
    actual_human_height=1.8  # Actual human height in meters
)

# Retarget a single frame
# smplx_data is a dict: {body_name: [position, quaternion], ...}
qpos = retarget.retarget(smplx_data)

# qpos format: [root_pos(3), root_rot(4), dof_pos(23)]
```

## Robot Specifications

- **Total DOF**: 29 (6 base + 23 joints)
- **Height** (default): ~0.5m at base
- **Joint breakdown**:
  - Waist: 1 DOF (yaw)
  - Legs: 5 DOF each (roll, pitch, yaw, knee, ankle) × 2 = 10 DOF
  - Arms: 6 DOF each (shoulder pitch/roll/yaw, elbow roll, wrist yaw/roll) × 2 = 12 DOF

## Known Issues & Solutions

1. **Corrupted STL meshes**: `l_wrist_roll_link.STL` and `r_wrist_roll_link.STL` were only 84 bytes and corrupted. Replaced with simple box geometries in the XML.

2. **Scaling adjustments**: The default scaling factors (0.6 for legs, 0.55 for arms) are initial estimates. You may need to tune these based on your specific human data to avoid:
   - Foot sliding
   - Unreachable IK targets
   - Joint limit violations

3. **Rotation offsets**: The quaternion offsets in the IK config align the human coordinate frame with the robot coordinate frame. These are set to reasonable defaults but may need adjustment depending on your specific use case.

## Tuning Guide

### If the robot is too small/large:
Edit `human_scale_table` in `smplx_to_dm_prototype_v02.json`

### If feet don't track well:
Adjust position/rotation weights in `ik_match_table2` for `l_foot_mocap` and `r_foot_mocap`

### If arms don't reach correctly:
1. Check arm scaling factors in `human_scale_table`
2. Adjust position weights in `ik_match_table2` for arm links
3. Consider tuning rotation offsets for shoulder/elbow links

### IK convergence issues:
Adjust in `GeneralMotionRetargeting.__init__()`:
- `damping`: Increase for more stable but slower convergence (default: 0.5)
- `max_iter`: Increase for better accuracy (default: 10)
- `solver`: Try different solvers ("daqp", "quadprog", etc.)

## Testing

Run the validation to ensure everything is set up correctly:

```python
import mujoco as mj
from general_motion_retargeting import GeneralMotionRetargeting as GMR

# Test XML loading
model = mj.MjModel.from_xml_path(
    "assets/dm_prototype_v02_zj01/xml/dm_prototype_v02_mocap.xml"
)
print(f"Model loaded: {model.nbody} bodies, {model.nv} DOF")

# Test GMR initialization
gmr = GMR(
    src_human="smplx",
    tgt_robot="dm_prototype_v02",
    actual_human_height=1.8
)
print(f"GMR initialized with {len(gmr.tasks1)} + {len(gmr.tasks2)} IK tasks")
```

## Credits

Created following the pattern from `unitree_g1` retargeting setup.
Based on the General Motion Retargeting (GMR) framework.


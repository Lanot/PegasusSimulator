# Local Isaac Sim 6.0.1 migration

Based on https://github.com/PegasusSimulator/PegasusSimulator/pull/144 and the iftahnaf/PegasusSimulator main reference at fcb99c0. Applied Python and extension manifest changes to local dev_6.0.1, retaining the existing optimized assets and params.py mappings. No upstream commits were merged or local commits created.

Changes include SimulationManager/RenderingManager lifecycle callbacks, experimental RigidPrim and Articulation wrappers instead of dynamic_control, updated camera/ROS/people imports, and migrated standalone examples. The manifest now declares version 6.0.1.

Additional local fixes: load_environment passes force_clear by keyword; vehicle attitude is read from the same physics pose as position and velocity to avoid stale USD transforms under Fabric.

## Check locally

Save your open scene and restart Isaac Sim to clear previously loaded modules. Enable Pegasus Simulator in Extensions. Confirm its displayed version is 6.0.1.

A separate headless smoke test (does not launch PX4 or ArduPilot):

```bash
/home/tolik/IsaacSim-6.0.1/python.sh /home/tolik/IsaacSim-6.0.1-Extensions/PegasusSimulator/tools/smoke_isaacsim_6.py
```

It checks extension enablement, ArduPilot configuration import, drone spawning and falling, upward force application, torque API access, and stop/start recreation of physics handles. Successful output ends with PEGASUS_SMOKE_PASS. The test exits nonzero on failure. It uses an empty scene without a floor, so falling below zero is expected.

ROS 2 requires a correctly initialized ROS runtime; the initial startup test reported missing rclpy and libament_index_cpp.so. Pegasus can run without that optional backend. ArduPilot SITL connection, motor mapping, controlled hover, and flight over the garden have not been verified by this test. People animation and camera/LiDAR behavior also need their own runtime checks.

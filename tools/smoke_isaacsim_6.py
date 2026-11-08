from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'extra_args': ['--ext-folder', str(Path(__file__).resolve().parents[1] / 'extensions')]})
import traceback
success = False
try:
 import omni.kit.app
 manager = omni.kit.app.get_app().get_extension_manager()
 manager.set_extension_enabled_immediate('pegasus.simulator', True)
 for _ in range(10): app.update()
 assert manager.is_extension_enabled('pegasus.simulator'), 'Pegasus extension did not enable'
 from pegasus.simulator.logic.backends import ArduPilotMavlinkBackendConfig
 from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
 from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
 from pegasus.simulator.params import ROBOTS
 import omni.usd, omni.timeline
 from pxr import UsdGeom
 omni.usd.get_context().new_stage()
 UsdGeom.SetStageMetersPerUnit(omni.usd.get_context().get_stage(), 1.0)
 UsdGeom.SetStageUpAxis(omni.usd.get_context().get_stage(), 'Z')
 PegasusInterface().initialize_world()
 cfg = MultirotorConfig()
 cfg.backends = []
 drone = Multirotor('/World/TestDrone', ROBOTS['Iris'], init_pos=[0.,0.,2.], config=cfg)
 timeline = omni.timeline.get_timeline_interface()
 timeline.play()
 for _ in range(60): app.update()
 print('PEGASUS_SMOKE_STATE', drone.state.position, flush=True)
 assert drone._body_rigid_prim is not None, 'Body wrapper not initialized'
 assert drone.state.position[2] < 2.0, 'Drone did not fall under gravity'
 # Apply upward force at every physics step, then verify vertical velocity rises.
 from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
 before = drone.state.linear_velocity[2]
 def thrust(dt, context):
  drone.apply_force([0., 0., 40.])
  drone.apply_torque([0., 0., 0.])
 cb = SimulationManager.register_callback(thrust, SimulationEvent.PHYSICS_PRE_STEP)
 for _ in range(20): app.update()
 SimulationManager.deregister_callback(cb)
 assert drone.state.linear_velocity[2] > before, 'Upward force failed to increase vertical velocity'
 timeline.stop()
 for _ in range(5): app.update()
 assert drone._body_rigid_prim is None, 'Physics handle not released on stop'
 timeline.play()
 for _ in range(20): app.update()
 assert drone._body_rigid_prim is not None, 'Physics handle not recreated on restart'
 timeline.stop()
 for _ in range(5): app.update()
 success = True
 print('PEGASUS_SMOKE_PASS', flush=True)
except Exception:
 traceback.print_exc()
 print('PEGASUS_SMOKE_FAIL', flush=True)
finally:
 app.close()

if not success:
 raise SystemExit(1)

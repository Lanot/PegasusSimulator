"""
| File: extension.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Implements the Pegasus_SimulatorExtension which omni.ext.IExt that is created when this class is enabled. In turn, this class initializes the extension widget.
"""
__all__ = ["Pegasus_SimulatorExtension"]

# Python garbage collenction and asyncronous API
import gc
import asyncio
from functools import partial
from threading import Timer

# Omniverse general API
import pxr
import carb
# BEGIN CUSTOM: Observe stage-open events for the PhysX preload.
import carb.eventdispatcher
# END CUSTOM: PhysX preload event dependency.
import omni.ext
import omni.usd
import omni.kit.ui
import omni.kit.app
import omni.ui as ui
import omni.timeline

from omni.kit.viewport.utility import get_active_viewport

# Pegasus Extension Files and API
from pegasus.simulator.params import MENU_PATH, WINDOW_TITLE
#from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Setting up the UI for the extension's Widget
from pegasus.simulator.ui.ui_window import WidgetWindow
from pegasus.simulator.ui.ui_delegate import UIDelegate

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class Pegasus_SimulatorExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):

        carb.log_info("Pegasus Simulator is starting up")

        # Save the extension id
        self._ext_id = ext_id

        # Create the UI of the app and its manager
        self.ui_delegate = None
        self.ui_window = None

        # Add the ability to show the window if the system requires it (QuickLayout feature)
        ui.Workspace.set_show_window_fn(WINDOW_TITLE, partial(self.show_window, None))

        # Add the extension to the editor menu inside isaac sim
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            self._menu = editor_menu.add_item(MENU_PATH, self.show_window, toggle=True, value=True)

        # Show the window (It call the self.show_window)
        ui.Workspace.show_window(WINDOW_TITLE, show=True)

        # BEGIN CUSTOM: Schedule PhysX preload when the saved MyPlot world opens.
        context = omni.usd.get_context()
        self._physx_preload_task = None
        # BEGIN CUSTOM: Track the attached Iris for cleanup on stage close.
        self._saved_iris = None
        self._saved_iris_close_observer = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=context.stage_event_name(omni.usd.StageEventType.CLOSING),
            on_event=self._detach_saved_iris,
            observer_name="pegasus.myplot_iris_close",
        )
        # END CUSTOM: Stage-scoped Iris runtime.
        self._physx_stage_observer = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=context.stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=self._schedule_physx_preload,
            observer_name="pegasus.myplot_physx_preload",
        )
        self._schedule_physx_preload()
        # END CUSTOM: Cover both new opens and the stage already open at startup.

        # BEGIN CUSTOM: Preload again after Stop resets the PhysX simulation.
        timeline = omni.timeline.get_timeline_interface()
        self._physx_stop_subscription = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self._schedule_physx_preload
        )
        # END CUSTOM: Prepare MyPlot for the next Play.

    # BEGIN CUSTOM: Preload stage physics before the normal Play action.
    def _schedule_physx_preload(self, event=None):
        if self._physx_preload_task is not None:
            self._physx_preload_task.cancel()
        self._physx_preload_task = asyncio.ensure_future(self._preload_physx())

    async def _preload_physx(self):
        from omni.physx import get_physx_interface
        from pxr import UsdPhysics

        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        if stage is None or not omni.timeline.get_timeline_interface().is_stopped():
            return

        scene = UsdPhysics.Scene.Get(stage, "/PhysicsScene")
        if stage.GetPrimAtPath("/World/MyPlotCollection") and scene and scene.GetPrim().IsActive():
            # BEGIN CUSTOM: Restore the saved Iris backend before Play.
            self._attach_saved_iris(stage)
            # END CUSTOM: Existing simulation callbacks launch SITL.
            get_physx_interface().force_load_physics_from_usd()
            carb.log_warning("Pegasus: preloaded MyPlot PhysX scene before Play.")
    # END CUSTOM: MyPlot stage-open preload.


    # BEGIN CUSTOM: Attach the saved MyPlot Iris to an ArduPilot SITL backend.
    def _attach_saved_iris(self, stage):
        from pegasus.simulator.logic.backends import ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
        from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
        from pxr import UsdPhysics

        path = "/World/MyDrones/quadrotor"
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsActive() or not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return

        pegasus = PegasusInterface()
        if pegasus.vehicle_manager.get_vehicle(path) is not None:
            return

        vehicle_id = 1
        config = MultirotorConfig()
        config.backends = [ArduPilotMavlinkBackend(config=ArduPilotMavlinkBackendConfig({
            "vehicle_id": vehicle_id,
            "ardupilot_autolaunch": True,
            "ardupilot_dir": pegasus.ardupilot_path,
            "ardupilot_vehicle_model": pegasus.ardupilot_default_airframe,
        }))]
        self._saved_iris = Multirotor(
            path, vehicle_id=vehicle_id, config=config, attach_existing=True
        )
        carb.log_warn(f"Pegasus: attached ArduPilot SITL to saved Iris at {path} (ID {vehicle_id}).")

    def _detach_saved_iris(self, event=None):
        vehicle, self._saved_iris = self._saved_iris, None
        if vehicle is not None:
            vehicle.close()
    # END CUSTOM: Release the live vehicle without deleting the saved prim.

    def show_window(self, menu, show):
        """
        Method that controls whether a widget window is created or not
        """

        if show == True:

            # Create a window and its delegate
            self.ui_delegate = UIDelegate()
            self.ui_window = WidgetWindow(self.ui_delegate)
            self.ui_window.set_visibility_changed_fn(self._visibility_changed_fn)

        # If we have a window and we are not supposed to show it, then change its visibility
        elif self.ui_window:
            self.ui_window.visible = False

    def _visibility_changed_fn(self, visible):
        """
        This method is invoked when the user pressed the "X" to close the extension window
        """

        # Update the Isaac sim menu visibility
        self._set_menu(visible)

        if not visible:
            # Destroy the window, because we create a new one in the show window method
            asyncio.ensure_future(self._destroy_window_async())

    def _set_menu(self, visible):
        """
        Method that updates the isaac sim ui menu to create the Widget window on and off
        """
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            editor_menu.set_value(MENU_PATH, visible)

    async def _destroy_window_async(self):

        # Wait one frame before it gets destructed (from NVidia example)
        await omni.kit.app.get_app().next_update_async()

        # Destroy the window UI if it exists
        if self.ui_window:
            self.ui_window.destroy()
            self.ui_window = None

    def on_shutdown(self):
        """
        Callback called when the extension is shutdown
        """
        carb.log_info("Pegasus Isaac extension shutdown")

        # BEGIN CUSTOM: Release the subscriptions and cancel any pending PhysX preload.
        self._physx_stage_observer = None
        self._physx_stop_subscription = None
        if self._physx_preload_task is not None:
            self._physx_preload_task.cancel()
            self._physx_preload_task = None
        # END CUSTOM: PhysX preload cleanup.

        # BEGIN CUSTOM: Release the attached Iris when Pegasus shuts down.
        self._saved_iris_close_observer = None
        self._detach_saved_iris()
        # END CUSTOM: Saved Iris runtime cleanup.

        # Destroy the isaac sim menu object
        self._menu = None

        # Destroy the window
        if self.ui_window:
            self.ui_window.destroy()
            self.ui_window = None

        # Destroy the UI delegate
        if self.ui_delegate:
            self.ui_delegate = None

        # De-register the function taht shows the window from the isaac sim ui
        ui.Workspace.set_show_window_fn(WINDOW_TITLE, None)

        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            editor_menu.remove_item(MENU_PATH)

        # Call the garbage collector
        gc.collect()

# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.maps import MAPs
import asyncio
import omni.ui as ui
from omni.isaac.ui.ui_utils import btn_builder


class MAPsExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="MAPs",
            title="Material Acceleration Platform AAU",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_hello_world.html",
            overview="This Example introduces the user on how to do cool stuff with Isaac Sim through scripting in asynchronous mode.",
            sample=MAPs(),
            file_path=os.path.abspath(__file__),
            number_of_extra_frames=3,
        )
        
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_simulation_controls_ui(frame)
        frame = self.get_frame(index=1)
        self.build_real_controls_ui(frame)
        frame = self.get_frame(index=2)
        self.build_experiment_controls_ui(frame)
        return
    
    def _on_sim_control_button_event(self):
        asyncio.ensure_future(self.sample._on_sim_control_event_async())
        self.task_ui_elements["Simulation Control"].enabled = False
        self.task_ui_elements["Real Setup Control"].enabled = True
        self.task_ui_elements["Connect PMC"].enabled = True
        self.task_ui_elements["Start Experiment"].enabled = True
        return
    
    def _on_real_control_button_event(self):
        asyncio.ensure_future(self.sample._on_real_control_event_async())
        self.task_ui_elements["Real Setup Control"].enabled = False
        self.task_ui_elements["Simulation Control"].enabled = True
        self.task_ui_elements["Connect PMC"].enabled = False
        self.task_ui_elements["Start Experiment"].enabled = True
        return
    
    def _on_connect_pmc_button_event(self):
        self.sample._connect_pmc()
        self.task_ui_elements["Real Setup Control"].enabled = True
        self.task_ui_elements["Simulation Control"].enabled = True
        self.task_ui_elements["Connect PMC"].enabled = False
        self.task_ui_elements["Start Experiment"].enabled = True
        return
    
    def _on_start_experiment_button_event(self):
        asyncio.ensure_future(self.sample._on_start_experiment_event_async())
        self.task_ui_elements["Real Setup Control"].enabled = False
        self.task_ui_elements["Simulation Control"].enabled = False
        self.task_ui_elements["Connect PMC"].enabled = False
        self.task_ui_elements["Start Experiment"].enabled = False
        return

    def post_reset_button_event(self):
        self.task_ui_elements["Simulation Control"].enabled = True
        self.task_ui_elements["Real Setup Control"].enabled = True
        self.task_ui_elements["Connect PMC"].enabled = True
        self.task_ui_elements["Start Experiment"].enabled = True
        return

    def post_load_button_event(self):
        self.task_ui_elements["Simulation Control"].enabled = True
        self.task_ui_elements["Real Setup Control"].enabled = False
        self.task_ui_elements["Connect PMC"].enabled = True
        self.task_ui_elements["Start Experiment"].enabled = True
        return

    def post_clear_button_event(self):
        self.task_ui_elements["Simulation Control"].enabled = False
        self.task_ui_elements["Real Setup Control"].enabled = False
        self.task_ui_elements["Connect PMC"].enabled = False
        self.task_ui_elements["Start Experiment"].enabled = False
        return

    def build_simulation_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Simulation"
                frame.visible = True
                dict = {
                    "label": "Simulation Control",
                    "type": "button",
                    "text": "Start Simulation",
                    "tooltip": "Simulation Control",
                    "on_clicked_fn": self._on_sim_control_button_event,
                }

                self.task_ui_elements["Simulation Control"] = btn_builder(**dict)
                self.task_ui_elements["Simulation Control"].enabled = False
    
    def build_real_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Real Setup"
                frame.visible = True

                dict = {
                    "label": "Connect PMC",
                    "type": "button",
                    "text": "Connect",
                    "tooltip": "Connect PMC",
                    "on_clicked_fn": self._on_connect_pmc_button_event,
                }
                self.task_ui_elements["Connect PMC"] = btn_builder(**dict)
                self.task_ui_elements["Connect PMC"].enabled = False


                dict = {
                    "label": "Real Setup Control",
                    "type": "button",
                    "text": "Start Real Setup",
                    "tooltip": "Real Setup Control",
                    "on_clicked_fn": self._on_real_control_button_event,
                }

                self.task_ui_elements["Real Setup Control"] = btn_builder(**dict)
                self.task_ui_elements["Real Setup Control"].enabled = False

    def build_experiment_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Experiment Control"
                frame.visible = True

                dict = {
                    "label": "Start Experiment",
                    "type": "button",
                    "text": "Start",
                    "tooltip": "Start Experiment",
                    "on_clicked_fn": self._on_start_experiment_button_event,
                }
                self.task_ui_elements["Start Experiment"] = btn_builder(**dict)
                self.task_ui_elements["Start Experiment"].enabled = False


                # dict = {
                #     "label": "Real Setup Control",
                #     "type": "button",
                #     "text": "Start Real Setup",
                #     "tooltip": "Real Setup Control",
                #     "on_clicked_fn": self._on_real_control_button_event,
                # }

                # self.task_ui_elements["Real Setup Control"] = btn_builder(**dict)
                # self.task_ui_elements["Real Setup Control"].enabled = False
       
        

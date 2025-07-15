import omni.ext
from isaacsim.gui.components.ui_utils import ui
import carb.events
import omni.timeline
import omni.physx
from omni.isaac.core.utils.stage import get_current_stage
import omni.kit.app

import asyncio
import random
import os
import sys
import pickle
import base64

from navsim_utils.sim_utils import TimeManager, GeospatialManager
from navsim_utils.extensions_utils import ExtensionUtils

project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

class RequestState:
    CANCELLED = "Cancelled"
    PENDING = "Pending"
    IN_PROGRESS = "In progress"
    COMPLETED = "Completed"

class USpaceClients(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()
        
    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_stop_sub = None
        self.on_play_sub = None
        self.event_sub = None
        if hasattr(self, "amazon_task"):    self.amazon_task.cancel()
        
    def on_physics_step(self, step_size:int):
        if self.is_sim_played:
            self.current_time += step_size

    def on_timeline_stop(self, event):
        self.current_time = 0
        self.is_sim_played = False
        self.time_manager.stop()
        self.amazon_task.cancel()
        self.amazon_new_request_timer = self.amazon_new_request_timer_base
        self.ui_amazon_new_req.text = f"New Request: {self.amazon_new_request_timer_base}"
        
    def on_timeline_play(self, event):
        is_resume = self.is_extension_on and self.is_sim_played
        is_played = not self.is_extension_on or not self.is_sim_played

        if is_resume:
            self.time_manager.resume()
            self.amazon_task = asyncio.ensure_future(self.start_amazon())
            return

        if is_played:
            random.seed(2)
            # 2 -> R5, R6, R25
            self.clients = {}
            self.amount_amazon_requests = 0
            self.vertiports_from_id, self.vertiports_from_pos = self.find_vertiports()

            # Clear UI containers
            self.ui_amazon_requests_container.clear()

            # Start the time manager
            self.time_manager.start()

            # Start the simulation
            self.is_sim_played = True
            self.amazon_task = asyncio.ensure_future(self.start_amazon())

    def on_timeline_pause(self, event):
        if self.is_extension_on:
            self.time_manager.pause()
            self.amazon_task.cancel()
    
    def init_vars(self):
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)

        self.timeline = omni.timeline.get_timeline_interface()
        self.on_stop_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)
        self.on_play_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.on_timeline_play)
        self.on_pause_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PAUSE), self.on_timeline_pause)
        
        self.event_stream = omni.kit.app.get_app().get_message_bus_event_stream()
        self.operator_event = carb.events.type_from_string("NavSim.Operator")
        self.uspace_clients_event = carb.events.type_from_string("NavSim.USpaceClients")
        self.event_sub = self.event_stream.create_subscription_to_push_by_type(self.uspace_clients_event, self.event_listener)
        
        self.amazon_new_request_timer_base = 10
        self.amazon_new_request_timer = self.amazon_new_request_timer_base
        self.amazon_id = "amazon"
        self.amount_amazon_requests = 0

        self.is_extension_on = False
        self.is_sim_played = False
        self.current_time = 0
        self.vertiports_from_id = {}
        self.clients = {}
        self.navsim_utils = ExtensionUtils()
        self.time_manager = TimeManager()
        self.geospatial_manager = GeospatialManager()

    def event_listener(self, event):
        if not event.payload["is_request"]:
            self.switch_on_off(event.payload["state"], is_from_event=True)
            return

        client_id = event.payload["client_id"]
        request_id = event.payload["request_id"]
        request_state = event.payload["state"]

        self.update_request_state(client_id, request_id, request_state)

    def build_ui(self):        
        self.window = ui.Window("CL: NavSim - clients", width=300, height=300, raster_policy=ui.RasterPolicy.NEVER)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=8, height=0):
                    # Title
                    ui.Spacer(height=10)
                    ui.Label(
                        "NAVSIM - CLIENTS", 
                        alignment=ui.Alignment.CENTER, 
                        style={"font_size": 20, "font_weight": "bold"}
                    )
                    ui.Spacer(height=5)

                    # On/Off button
                    self.on_off_button = ui.ToolButton(
                        text="ON", 
                        height=30, 
                        clicked_fn=lambda state=False, is_from_event=False: self.switch_on_off(state, is_from_event), 
                        style={"background_color": ui.color("#6f9523")}
                    )

                    # Amazon client
                    with ui.ZStack(height=200):
                        ui.Rectangle(style={"background_color": 0xFF303030, 
                                            "border_radius": 10, 
                                            "corner_flag": ui.CornerFlag.ALL})
                        
                        ui.Label("AMAZON", alignment=ui.Alignment.LEFT_CENTER, 
                                style={"font_size": 14, "font_weight": "bold", "margin_width": 10})
                        
                        with ui.VStack(height=50, style={"margin_width": 30}):
                            ui.Label("Requests", alignment=ui.Alignment.CENTER_TOP,
                                    style={"font_size": 14, "font_weight": "bold", "margin_height": 5})

                            with ui.ZStack():
                                ui.Rectangle(height=150, style={"background_color": 0xFF5b5b5b, 
                                            "border_radius": 10, 
                                            "corner_flag": ui.CornerFlag.ALL})
                                
                                self.ui_amazon_requests_scrolling_frame = ui.ScrollingFrame(
                                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                    style={"background_color": 0xFF5b5b5b, "margin":7}, height=150
                                )
                                
                                with self.ui_amazon_requests_scrolling_frame:
                                    self.ui_amazon_requests_container = ui.VStack(height=0)

                            self.ui_amazon_new_req = ui.Label(f"New Request: {self.amazon_new_request_timer_base}", alignment=ui.Alignment.CENTER_BOTTOM, 
                                    style={"font_size": 12, "margin": 10})

                    ui.Spacer(height=30)
                    # Force new request
                    with ui.HStack():
                        ui.Label("Client ID")
                        self.ui_client_id = ui.StringField()
                        self.ui_client_id.model.set_value("amazon")
                    with ui.HStack():
                        ui.Label("Request ID")
                        self.ui_request_id = ui.StringField()
                        self.ui_request_id.model.set_value("request_1000")
                    with ui.HStack():
                        ui.Label("Init time")
                        self.ui_init_time = ui.StringField()
                    with ui.HStack():
                        ui.Label("End time")
                        self.ui_end_time = ui.StringField()
                    with ui.HStack():
                        ui.Label("Origin")
                        self.ui_origin = ui.StringField()
                        self.ui_origin.model.set_value("v_2")
                    with ui.HStack():
                        ui.Label("Destination")
                        self.ui_destination = ui.StringField()
                        self.ui_destination.model.set_value("v_12")

                    ui.Button("Send request", height=50, clicked_fn=self.send_request_by_hand)

    def switch_on_off(self, state, is_from_event):
        model = self.on_off_button.model
        model_value = model.get_value_as_bool()
        
        if is_from_event:
            internal_state = state
            model.set_value(state)
        else:
            internal_state = model_value

        if internal_state:
            self.switch_extension_state(on=True, is_from_event=is_from_event)

            style={"background_color": ui.color("#952323")}
            self.on_off_button.set_style(style)
            self.on_off_button.text = "OFF"

        else:
            self.switch_extension_state(on=False, is_from_event=is_from_event)
            
            style={"background_color": ui.color("#6f9523")}
            self.on_off_button.set_style(style)
            self.on_off_button.text = "ON"

    def switch_extension_state(self, on, is_from_event):
        # Update internal state
        self.is_extension_on = on

        if not is_from_event:
            # Update Operator extension state
            self.event_stream.push(
                self.operator_event, 
                payload={
                    "is_request": False, 
                    "state": on
                }
            )

    def send_request_by_hand(self):
        client_id = self.ui_client_id.model.get_value_as_string()
        request_id = self.ui_request_id.model.get_value_as_string()
        state = RequestState.PENDING
        init_time = self.ui_init_time.model.get_value_as_int()
        end_time = self.ui_end_time.model.get_value_as_int()
        origin = self.ui_origin.model.get_value_as_string()
        destination = self.ui_destination.model.get_value_as_string()
        begin_time = self.current_time
        finish_time = "None"

        request = self.process_request(
            client_id,
            request_id,
            state,
            init_time,
            end_time,
            origin,
            destination,
            begin_time,
            finish_time
        )
        self.event_stream.push(self.operator_event, payload=request)

    def find_vertiports(self):
        vertiports_prims = self.navsim_utils.get_vertiport_prims()
        vertiports_from_id = {}
        vertiports_from_pos = {}
        
        for prim in vertiports_prims:
            id = prim.GetAttribute("NavSim:id").Get()
            position = prim.GetAttribute("xformOp:translate").Get()
            model = prim.GetAttribute("NavSim:model").Get()

            vertiports_from_id[id] = {"position": position, "model": model}
            vertiports_from_pos[position] = id

        return vertiports_from_id, vertiports_from_pos

    def update_request_state(self, client_id, request_id, state):
        is_terminated = state in (RequestState.CANCELLED, RequestState.COMPLETED)

        if is_terminated:
            time = self.time_manager.sim_to_real(self.current_time)
            self.clients[client_id][request_id]["request_finish_time"] = time

        self.clients[client_id][request_id]["state"] = state
        self.print_requests(client_id)

    def print_requests(self, client):
        match client:
            case "amazon":
                self.ui_amazon_requests_container.clear()

            case _:
                pass

        for key, value in self.clients[client].items():
            self.print_new_request(
                client,
                key, 
                value["state"],
                value["init_time"],
                value["end_time"],
                value["origin"],
                value["destination"],
                value["request_begin_time"],
                value["request_finish_time"]
            )

    def print_new_request(self, client, request_id, state, init_time, end_time, 
                          origin, destination, request_begin_time, request_finish_time):
        match client:
            case "amazon":
                container = self.ui_amazon_requests_container
            
            case _:
                pass

        request_id_label = ui.Label(f"Request ID: {request_id}\n")
        state_label = ui.Label(f"State: {state}\n")
        init_time_label = ui.Label(f"Init time: {init_time}\n")
        end_time_label = ui.Label(f"End time: {end_time}\n")
        origin_label = ui.Label(f"Origin: {origin}\n")
        destination_label = ui.Label(f"Destination: {destination}\n")
        request_begin_time_label = ui.Label(f"Request begin time: {request_begin_time}\n")
        request_finish_time_label = ui.Label(f"Request finish time: {request_finish_time}\n")
        spacer = ui.Spacer(height=10)

        container.add_child(request_id_label)
        container.add_child(state_label)
        container.add_child(init_time_label)
        container.add_child(end_time_label)
        container.add_child(origin_label)
        container.add_child(destination_label)
        container.add_child(request_begin_time_label)
        container.add_child(request_finish_time_label)
        container.add_child(spacer)

    async def start_amazon(self):
        while self.is_sim_played:
            await asyncio.sleep(1)

            # Avoid exception when saving file while running simulation
            if not hasattr(self, "is_sim_played"):
                break

            self.amazon_new_request_timer -= 1
            self.ui_amazon_new_req.text = f"New Request: {self.amazon_new_request_timer}"

            if self.amazon_new_request_timer == 0:                
                request = self.create_request(self.amazon_id)
                self.event_stream.push(self.operator_event, payload=request)
                
                self.reset_new_request_timer(self.amazon_id)

    def create_request(self, client_id):
        self.amount_amazon_requests += 1

        request_id = f"request_{self.amount_amazon_requests}"
        state = RequestState.PENDING
        begin_time = self.current_time
        finish_time = "None"

        init_time = begin_time + random.randint(30, 60)
        end_time = init_time + random.randint(30, 60)

        vertiport_ids = list(self.vertiports_from_id.keys())
        origin = random.choice(vertiport_ids)
        destination = random.choice(vertiport_ids)

        while destination == origin:
            destination = random.choice(vertiport_ids)

        request = self.process_request(
            client_id,
            request_id,
            state,
            init_time,
            end_time,
            origin,
            destination,
            begin_time,
            finish_time
        )
        return request
                   
    def process_request(self, client_id, request_id, state, init_time, end_time, origin, 
                         destination, request_begin_time, request_finish_time):
        # Convert times to real time
        real_begin_time = self.time_manager.sim_to_real(request_begin_time)
        real_init_time = self.time_manager.sim_to_real(init_time)
        real_end_time = self.time_manager.sim_to_real(end_time)

        self.register_request(
            client_id, 
            request_id,
            state,
            real_init_time, 
            real_end_time, 
            origin, 
            destination, 
            real_begin_time, 
            request_finish_time
        )

        # Build the request payload
        serialized_begin_time = base64.b64encode(pickle.dumps(real_begin_time)).decode('utf-8')
        serialized_init_time = base64.b64encode(pickle.dumps(real_init_time)).decode('utf-8')
        serialized_end_time = base64.b64encode(pickle.dumps(real_end_time)).decode('utf-8')

        request = {
            "is_request": True,
            "sender": "client",
            "client_id": client_id,
            "request_id": request_id,
            "init_time": serialized_init_time,
            "end_time": serialized_end_time,
            "origin": origin,
            "destination": destination,
            "request_begin_time": serialized_begin_time,
            "request_finish_time": request_finish_time
        }

        return request

    def register_request(self, client_id, request_id, state, init_time, end_time, origin, 
                         destination, request_begin_time, request_finish_time):
        if client_id not in self.clients:
            self.clients[client_id] = {}

        self.clients[client_id][request_id] = {
            "state": state,
            "init_time": init_time,
            "end_time": end_time,
            "origin": origin,
            "destination": destination,
            "request_begin_time": request_begin_time,
            "request_finish_time": request_finish_time
        }

        self.print_new_request(
            client_id,
            request_id,
            state,
            init_time,
            end_time,
            origin,
            destination,
            request_begin_time,
            request_finish_time
        )
    
    def reset_new_request_timer(self, client):
        match client:
            case "amazon":
                self.amazon_new_request_timer = self.amazon_new_request_timer_base
                var = random.randint(-5, 5)
                self.amazon_new_request_timer += var
                self.ui_amazon_new_req.text = f"New Request: {self.amazon_new_request_timer}"

            case _:
                pass
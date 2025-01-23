import omni.ext
import omni.ui as ui
import carb.events
import omni.timeline
import omni.physx
from omni.isaac.core.utils.stage import get_current_stage
import omni.kit.app

import asyncio
import random
import os
import sys

from navsim_utils.extensions_utils import ExtensionUtils

project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

class RequestState:
    PENDING = "pending"
    COMPLETED = "completed"

class USpaceClients(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()
        
    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_stop_sub = None
        self.on_play_sub = None
        self.event_sub = None

    def on_physics_step(self, step_size:int):
        self.current_time += step_size

    def on_timeline_stop(self, event):
        self.current_time = 0
        self.is_simulating = False
        self.amazon_new_request_timer = self.amazon_new_request_timer_base
        self.ui_amazon_new_req.text = f"New Request: {self.amazon_new_request_timer_base}"
        
    def on_timeline_play(self, event):
        random.seed(2)
        # 2 -> R5, R6, R25
        self.clients = {}
        self.amount_amazon_requests = 0
        self.vertiports_from_id, self.vertiports_from_pos = self.find_vertiports()

        self.build_ui()
        self.start_simulation()
    
    def init_vars(self):
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)

        self.timeline = omni.timeline.get_timeline_interface()
        self.on_stop_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)
        self.on_play_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.on_timeline_play)
        
        self.event_stream = omni.kit.app.get_app().get_message_bus_event_stream()
        self.operator_event = carb.events.type_from_string("NavSim.Operator")
        self.uspace_clients_event = carb.events.type_from_string("NavSim.USpaceClients")
        self.event_sub = self.event_stream.create_subscription_to_push_by_type(self.uspace_clients_event, self.event_listener)
        
        self.amazon_new_request_timer_base = 30
        self.amazon_new_request_timer = self.amazon_new_request_timer_base
        self.amazon_id = "amazon"
        self.amount_amazon_requests = 0

        self.current_time = 0
        self.is_simulating = False
        self.vertiports_from_id = {}
        self.clients = {}
        self.navsim_utils = ExtensionUtils()

    def event_listener(self, event):
        client_id = event.payload["client_id"]
        request_id = event.payload["request_id"]
        request_state = event.payload["state"]

        self.update_request_state(client_id, request_id, request_state)

    def build_ui(self):        
        self.window = ui.Window("CL: NavSim - clients", width=300, height=300, raster_policy=ui.RasterPolicy.NEVER)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=10, height=0):
                    ui.Spacer(height=10)
                    # Info
                    ui.Label("NAVSIM - CLIENTS", alignment=ui.Alignment.CENTER, style={"font_size": 20, "font_weight": "bold"})

                    ui.Spacer(height=5)

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
                                
                                with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                            style={"background_color": 0xFF5b5b5b, "margin":7}, height=150):
                                    self.ui_amazon_requests = ui.Label("", alignment=ui.Alignment.LEFT)

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

    def send_request_by_hand(self):
        client_id = self.ui_client_id.model.get_value_as_string()
        request_id = self.ui_request_id.model.get_value_as_string()
        init_time = self.ui_init_time.model.get_value_as_int()
        end_time = self.ui_end_time.model.get_value_as_int()
        origin = self.ui_origin.model.get_value_as_string()
        destination = self.ui_destination.model.get_value_as_string()
        begin_time = round(self.current_time, 2)
        finish_time = "In progress"

        request = {
            "sender": "client",
            "client_id": client_id,
            "request_id": request_id,
            "init_time": init_time,
            "end_time": end_time,
            "origin": origin,
            "destination": destination,
            "request_begin_time": begin_time,
            "request_finish_time": finish_time
        }

        self.event_stream.push(self.operator_event, payload=request)
        self.register_request(client_id, request)

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
        if state == RequestState.COMPLETED:
            self.clients[client_id][request_id]["request_finish_time"] = round(self.current_time, 2)

        self.print_requests(client_id)

    def print_requests(self, client):
        match client:
            case "amazon":
                final_string = ""
                for key, value in self.clients[client].items():
                    string = "Request ID: " + key + "\n"
                    string += "Init time: " + str(value["init_time"]) + "\n"
                    string += "End time: " + str(value["end_time"]) + "\n"
                    string += "Origin: " + str(value["origin"]) + "\n"
                    string += "Destination: " + str(value["destination"]) + "\n"
                    string += "Request begin time: " + str(value["request_begin_time"]) + "\n"
                    string += "Request finish time: " + str(value["request_finish_time"]) + "\n"
                    string += "\n"

                    final_string += string

                self.ui_amazon_requests.text = final_string

            case _:
                pass

    def start_simulation(self):
        if not self.is_simulating:
            self.is_simulating = True

            asyncio.ensure_future(self.start_amazon())

    async def start_amazon(self):
        while self.is_simulating:
            await asyncio.sleep(1)

            # Avoid exception when saving file while running simulation
            if not hasattr(self, "is_simulating"):
                break

            self.amazon_new_request_timer -= 1
            self.ui_amazon_new_req.text = f"New Request: {self.amazon_new_request_timer}"

            if self.amazon_new_request_timer == 0:                
                request = self.create_request(self.amazon_id)
                self.event_stream.push(self.operator_event, payload=request)

                self.register_request(self.amazon_id, request)
                self.reset_new_request_timer(self.amazon_id)

    def create_request(self, client_id):
        request_id = f"request_{self.amount_amazon_requests}"
        begin_time = round(self.current_time, 2)
        finish_time = "In progress"

        init_time = round(begin_time + random.randint(30, 60), 2)
        end_time = round(init_time + random.randint(30, 60), 2)

        vertiport_ids = list(self.vertiports_from_id.keys())
        origin = random.choice(vertiport_ids)
        destination = random.choice(vertiport_ids)

        while destination == origin:
            destination = random.choice(vertiport_ids)

        request = {
            "sender": "client",
            "client_id": client_id,
            "request_id": request_id,
            "init_time": init_time,
            "end_time": end_time,
            "origin": origin,
            "destination": destination,
            "request_begin_time": begin_time,
            "request_finish_time": finish_time
        }

        self.amount_amazon_requests += 1

        return request
        
            
    def register_request(self, client_id, request):
        if client_id not in self.clients:
            self.clients[client_id] = {}

        self.clients[client_id][request["request_id"]] = {
            "init_time": request["init_time"],
            "end_time": request["end_time"],
            "origin": request["origin"],
            "destination": request["destination"],
            "request_begin_time": request["request_begin_time"],
            "request_finish_time": request["request_finish_time"]
        }
        self.print_requests(self.amazon_id)
    
    def reset_new_request_timer(self, client):
        match client:
            case "amazon":
                self.amazon_new_request_timer = self.amazon_new_request_timer_base
                var = random.randint(-5, 5)
                self.amazon_new_request_timer += var
                self.ui_amazon_new_req.text = f"New Request: {self.amazon_new_request_timer}"

            case _:
                pass
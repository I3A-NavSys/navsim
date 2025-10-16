import omni.ext
from isaacsim.gui.components.ui_utils import ui
import carb.events
import omni.kit.app

import pickle
import base64

from navsim_utils.sim_utils import *
from navsim_utils.extensions_utils import ExtensionUtils


class USpaceVertiportOperator(omni.ext.IExt):
    def on_startup(self, ext_id) -> None:
        self.init_vars()
        self.build_ui()

    def init_vars(self) -> None:
        # Get event stream and initialize event variables
        self.event_stream = omni.kit.app.get_app().get_message_bus_event_stream()
        self.operator_vertiport_event = carb.events.type_from_string("NavSim.OperatorVertiport")
        self.uspace_clients_event = carb.events.type_from_string(
            "NavSim.USpaceClients")
        
        self.event_sub = self.event_stream.create_subscription_to_push_by_type(self.operator_vertiport_event, self.event_listener)

        # Initialize vars
        self.pads = {}
        self.init_pads()
        self.extension_utils = ExtensionUtils()

    def on_shutdown(self) -> None:
        # Kill event stream subscribers
        self.event_sub = None

    def init_pads(self) -> None:
        pad_prims = self.extension_utils.get_public_vertiport_prims()

        # Extract data from vertiport prims
        for prim in pad_prims:
            id = prim.GetAttribute("NavSim:id").Get()
            pos = prim.GetAttribute("xformOp:translate").Get()
            model = prim.GetAttribute("NavSim:model").Get()
            self.pads[id] = {"position": pos, "model": model}  

    def update_pad_state(self, id, state) -> None:
        pass

    def event_listener(self, event) -> None:
        payload = event.payload

        # Check payload message type
        match payload["type_message"]:
            case TypeMessage.USPACE:
                self.handle_uspace_msg(payload)

    def handle_uspace_msg(self, payload) -> None:
        msg = payload["msg"]

        match msg["sender"]:
            case TypeSender.USPACE_CLIENT:
                self.inform_client(
                    TypeMessage.USPACE
                )

    def inform_client(self, type_message) -> None:
        payload = {"type_message": type_message}
        msg = {"sender": TypeSender.OPERATOR_VERTIPORT}

        match type_message:
            case TypeMessage.USPACE:
                vertiports_from_id, _ = self.get_vertiports()
                msg["vertiports"] = base64.b64encode(pickle.dumps(vertiports_from_id)).decode('utf-8')

        payload["msg"] = msg

        self.event_stream.push(self.uspace_clients_event, payload=payload)

    def build_ui(self) -> None:
        pass

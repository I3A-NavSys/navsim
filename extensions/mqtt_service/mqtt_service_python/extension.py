import paho.mqtt.client as mqtt_client


import omni.ext
# from isaacsim.gui.components.ui_utils import ui
from isaacsim.gui.components import ui
from omni.ui import color as cl
import omni.timeline


from navsim_utils.extensions_utils import ExtensionUtils


class MQTTService(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()

    def on_shutdown(self):
        self.op_stop_sub = None
        self.on_play_sub = None

    def on_physics_step(self, step_size: int):
        pass

    def on_timeline_stop(self, event):
        pass

    def on_timeline_play(self, event):
        pass

    # ----------------------------------
    # -------- INITIALIZATION ----------
    # ----------------------------------
    def init_vars(self):
        self.timeline = omni.timeline.get_timeline_interface()
        timeline_stream = self.timeline.get_timeline_event_stream()
        self.on_stop_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), 
            self.on_timeline_stop
        )
        self.on_play_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), 
            self.on_timeline_play
        )
        
        self.broker_address = "localhost"
        self.broker_port = 1883
        self.client_id = "NavSim_mqtt_service"
        self.client = mqtt_client.Client(
            mqtt_client.CallbackAPIVersion.VERSION2, 
            self.client_id
        )
        self.client.on_message = self.on_message
        self.is_connected = False
        self.subscribed_topics = []
        
        self.extension_utils = ExtensionUtils()

    # ----------------------------------
    # ---- UI BUILDING AND HANDLING ----
    # ----------------------------------
    def update_connection_status(self):
        if self.is_connected:
            self.ui_connection_status_container.style = {
                "background_color": self.extension_utils.colors["G"],
                "border_radius": 10
            }
            self.ui_connection_status_label.text = "CONNECTED"
        else:
            self.ui_connection_status_container.style = {
                "background_color": self.extension_utils.colors["R"],
                "border_radius": 10
            }
            self.ui_connection_status_label.text = "DISCONNECTED"
    
    def on_message(self, client, userdata, msg):
        print(f"Received message: {msg.payload.decode()} on topic: {msg.topic}")

    def connect_client(self):
        if not self.is_connected:
            self.broker_address = self.ui_broker_address_field.model.get_value_as_string()
            self.broker_port = self.ui_broker_port_field.model.get_value_as_int()
            result = self.client.connect(self.broker_address, self.broker_port)

            if result == 0:
                print("Connected to MQTT Broker!")
                self.client.loop_start()
                self.is_connected = True
                self.update_connection_status()

    def disconnect_client(self):
        if self.is_connected:
            print("Disconnected from MQTT Broker!")
            self.client.loop_stop()
            self.client.disconnect()
            self.is_connected = False
            self.update_connection_status()

    def print_topics(self):
        self.ui_subscribed_topics_container.clear()
        
        with self.ui_subscribed_topics_container:
            for topic in self.subscribed_topics:
                with ui.ZStack():
                    ui.Rectangle(
                        style={
                            "background_color": cl("#757575"), 
                            "border_radius": 5
                        }
                    )
                    
                    with ui.HStack():
                        ui.Label(
                            topic, 
                            word_wrap=True, 
                            style={"margin_width": 10, "margin_height": 5}
                        )
                        ui.Button(
                            text="REMOVE",
                            width=100,
                            clicked_fn=lambda t=topic: self.remove_topic(t),
                            style={"margin_width": 10, "margin_height": 5},
                        )

    def add_topic(self):
        new_topic = self.ui_add_topic_field.model.get_value_as_string()
        
        if new_topic in self.subscribed_topics or new_topic == "":
            return
        
        self.client.subscribe(new_topic)
        self.subscribed_topics.append(new_topic)
        self.print_topics()
        
    def remove_topic(self, topic):
        self.client.unsubscribe(topic)
        self.subscribed_topics.remove(topic)
        self.print_topics()
    
    def publish_message(self):
        if self.is_connected:
            topic = self.ui_publish_topic_field.model.get_value_as_string()
            message = self.ui_publish_message_field.model.get_value_as_string()
            
            if topic == "" and message == "":
                return
            
            self.client.publish(topic, message)
    
    def build_ui(self):
        self.window = ui.Window("MQTT: NavSim - MQTT Service", width=300, height=300)
        self.window.deferred_dock_in("Layers")
        self.window.frame.set_style(self.extension_utils.Window_dark_style)

        with self.window.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(style=self.extension_utils.VStack_A, height=0):
                    # Title
                    ui.Spacer(height=10)
                    ui.Label(
                        "NAVSIM - MQTT Service",
                        alignment=ui.Alignment.CENTER,
                        style={"font_size": 20, "font_weight": "bold"},
                    )
                    
                    ui.Spacer(height=20)
                    
                    # Connection status
                    with ui.ZStack():
                        self.ui_connection_status_container = ui.Rectangle(
                            height=30,
                            alignment=ui.Alignment.CENTER,
                            style={
                                "background_color": self.extension_utils.colors["R"], 
                                "border_radius": 10
                            }
                        )
                        self.ui_connection_status_label = ui.Label(
                            "DISCONNECTED",
                            alignment=ui.Alignment.CENTER,
                            style={"color": 0xFFFFFFFF}
                        )
                        
                    ui.Spacer(height=30)

                    # Broker information
                    with ui.ZStack(style={"margin_width": 10}):
                        ui.Rectangle(
                            height=100,
                            alignment=ui.Alignment.CENTER,
                            style={
                                "background_color": 0xFF5B5B5B, 
                                "border_radius": 10
                            }
                        )
                        
                        with ui.VStack(height=0, spacing=self.extension_utils.SPACING_S):
                            ui.Spacer(height=5)
                            ui.Label("Broker information", alignment=ui.Alignment.CENTER)
                            
                            with ui.HStack():
                                ui.Label("Address:")
                                ui.Label("Port:", width=100)
                                
                            with ui.HStack():
                                self.ui_broker_address_field = ui.StringField(height=0)
                                self.ui_broker_port_field = ui.IntField(height=0, width=100)
                                
                                self.ui_broker_address_field.model.set_value(self.broker_address)
                                self.ui_broker_port_field.model.set_value(self.broker_port)
                                
                    ui.Spacer(height=20)
                    
                    # Topic information
                    with ui.ZStack():
                        ui.Rectangle(
                            style={
                                "background_color": 0xFF5B5B5B, 
                                "border_radius": 10
                            }
                        )
                        
                        with ui.VStack(height=0, spacing=self.extension_utils.SPACING_S):
                            ui.Spacer(height=5)
                            ui.Label("Topic information", alignment=ui.Alignment.CENTER)
                            
                            with ui.HStack():
                                ui.Label("Add topic:")
                                
                            with ui.HStack():
                                self.ui_add_topic_field = ui.StringField(height=30)
                                self.ui_add_topic_button = ui.Button(
                                    text="ADD",
                                    width=75,
                                    clicked_fn=self.add_topic,
                                )
                            
                            ui.Spacer(height=10)
                            
                            ui.Label("Subscribed topics:")
                            with ui.ScrollingFrame(
                                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                style=self.extension_utils.ScrollingFrame_style,
                                height=220
                            ):
                                
                                self.ui_subscribed_topics_container = ui.VStack(
                                    height=0, 
                                    style={"margin_height": 5}
                                )
                                
                            ui.Spacer(height=10)
                        
                                
                    ui.Spacer(height=20)
                    
                    # Connect/Disconnect buttons
                    with ui.HStack():
                        self.connect_button = ui.Button(
                            text="CONNECT",
                            height=40,
                            style={
                                "font_size": 16, 
                                "font_weight": "bold", 
                                "background_color": self.extension_utils.colors["G"],
                                ":hovered": {"background_color": cl("#A3A3A3")},
                            },
                            clicked_fn=self.connect_client,
                        )
                        self.disconnect_button = ui.Button(
                            text="DISCONNECT",
                            height=40,
                            style={
                                "font_size": 16, 
                                "font_weight": "bold", 
                                "background_color": self.extension_utils.colors["R"],
                                ":hovered": {"background_color": cl("#A3A3A3")},
                            },
                            clicked_fn=self.disconnect_client,
                        )
                        
                    ui.Spacer(height=20)
                        
                    # Test validation block
                    with ui.ZStack():
                        ui.Rectangle(
                            style={
                                "background_color": 0xFF5B5B5B, 
                                "border_radius": 10
                            }
                        )
                        
                        with ui.VStack(height=0, spacing=self.extension_utils.SPACING_S):
                            ui.Spacer(height=5)
                            ui.Label("Test validation", alignment=ui.Alignment.CENTER)

                            ui.Label("Topic:")
                            self.ui_publish_topic_field = ui.StringField()
                            
                            ui.Label("Message:")
                            self.ui_publish_message_field = ui.StringField()

                            ui.Spacer(height=10)

                            ui.Button(
                                text="PUBLISH",
                                height=50,
                                clicked_fn=self.publish_message
                            )
                            
                            ui.Spacer(height=5)
        
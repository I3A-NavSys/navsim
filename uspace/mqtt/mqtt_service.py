import os
import paho.mqtt.client as mqtt_client

class MQTTService:
    broker_address = os.getenv("HOST_ADDRESS", "localhost")
    broker_port = int(os.getenv("HOST_PORT", 1883))
         
    def build_client(client_id: str):
        return mqtt_client.Client(
            mqtt_client.CallbackAPIVersion.VERSION2, 
            client_id
        )

    def connect_client(client):
        result = client.connect(MQTTService.broker_address, MQTTService.broker_port)

        if result == 0:
            client.loop_start()
            return True
        
        return False

    def disconnect_client(client):
            client.loop_stop()
            client.disconnect()
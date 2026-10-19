import paho.mqtt.client as mqtt_client

class MQTTService:         
    def build_client(client_id: str):
        return mqtt_client.Client(
            mqtt_client.CallbackAPIVersion.VERSION2, 
            client_id
        )

    def connect_client(client, host, port):
        result = client.connect(host, port)

        if result == 0:
            client.loop_start()
            return True
        
        return False

    def disconnect_client(client):
            client.loop_stop()
            client.disconnect()
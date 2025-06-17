import time
from umqtt.simple import MQTTClient

class AdafruitIOClient:
    IO_SERVER = "io.adafruit.com"
    IO_PORT   = 1883

    def __init__(self, username: str, key: str, wifi_manager, device_id: str = None):
        self.username = username
        self.key = key
        self.wifi = wifi_manager
        self.client_id = device_id or username + "_client"
        self._client = None
        self._callbacks = {}

    def connect(self, timeout: int = 15) -> bool:
        """Ensure WiFi is connected, then connect to Adafruit IO via MQTT."""
        if not self.wifi.is_connected():
            if not self.wifi.connect():
                return False
        self._client = MQTTClient(
            client_id=self.client_id,
            server=self.IO_SERVER,
            port=self.IO_PORT,
            user=self.username,
            password=self.key,
            keepalive=60
        )
        # set a generic callback dispatcher
        self._client.set_callback(self._mqtt_callback)
        try:
            self._client.connect()
            return True
        except Exception as e:
            print("MQTT connect error:", e)
            return False

    def _mqtt_callback(self, topic: bytes, msg: bytes):
        """Internal dispatcher for incoming messages."""
        # topic format: <username>/feeds/<feed_name>
        parts = topic.decode().split('/')
        if len(parts) >= 3 and parts[-2] == 'feeds':
            feed = parts[-1]
            if feed in self._callbacks:
                try:
                    self._callbacks[feed](msg.decode())
                except Exception as e:
                    print(f"Callback error for {feed}:", e)

    def publish(self, feed: str, value) -> None:
        topic = f"{self.username}/feeds/{feed}".encode()
        payload = str(value).encode()
        self._client.publish(topic, payload)

    def subscribe(self, feed: str, callback) -> None:
        """Subscribe to a feed and register a callback(msg)
            Callback receives the payload as string"""
        topic = f"{self.username}/feeds/{feed}".encode()
        self._callbacks[feed] = callback
        self._client.subscribe(topic)

    def check_msg(self):
        """ non-blocking """
        # must be called frequently in main loop
        try:
            self._client.check_msg()
        except Exception as e:
            print("Error checking MQTT messages:", e)

    def disconnect(self):
        try:
            self._client.disconnect()
        except:
            pass



### Example usage:
# wifi = WiFiManager('Mob', 'Moldova1')
# aio = AdafruitIOClient('your_username', 'your_key', wifi)
# if aio.connect():
#     aio.publish('light', 123)
#     def on_cmd(val): print('Command:', val)
#     aio.subscribe('command', on_cmd)
#     while True:
#         aio.check_msg()
#         time.sleep(1)

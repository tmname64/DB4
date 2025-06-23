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
        parts = topic.decode().split('/')
        if len(parts) >= 3 and parts[-2] == 'feeds':
            feed = parts[-1]
            if feed in self._callbacks:
                try:
                    self._callbacks[feed](msg.decode())
                except Exception as e:
                    print(f"Callback error for {feed}:", e)

    def publish(self, feed: str, value) -> None:
        """Publish a value to a specific feed."""
        topic = f"{self.username}/feeds/{feed}".encode()
        payload = str(value).encode()
        self._client.publish(topic, payload)
        print(f"Published {feed} → {value}")

    def subscribe(self, feed: str, callback) -> None:
        """Subscribe to a feed and register a callback(msg)."""
        topic = f"{self.username}/feeds/{feed}".encode()
        self._callbacks[feed] = callback
        self._client.subscribe(topic)
        print(f"Subscribed to feed '{feed}'")

    def check_msg(self):
        """Non-blocking check for incoming messages."""
        try:
            self._client.check_msg()
        except Exception as e:
            print("Error checking MQTT messages:", e)

    def disconnect(self):
        """Disconnect the MQTT client."""
        try:
            self._client.disconnect()
        except:
            pass

    # ------- Helpful data-sending functions -------
    def send_system_status(self, running: bool) -> None:
        """Publish system running status (True/False)."""
        self.publish('system', int(running))

    def send_temperature(self, temperature: float) -> None:
        """Publish current temperature reading."""
        self.publish('temperature', temperature)

    def send_od(self, od_value: float) -> None:
        """Publish optical density (OD) reading."""
        self.publish('od', od_value)

    def send_pump_status(self, pump1: int, pump2: int = None) -> None:
        """Publish pump speeds/frequencies for pump1 and optional pump2."""
        self.publish('pump1', pump1)
        if pump2 is not None:
            self.publish('pump2', pump2)

    def send_cooler_status(self, status: str) -> None:
        """Publish cooler status (e.g., 'ON', 'OFF', 'LOW')."""
        self.publish('cooler', status)

    def send_concentration(self, concentration: float) -> None:
        """Publish concentration measurement."""
        self.publish('concentration', concentration)

    def send_temperature_over_time(self, temp_time: float) -> None:
        """Publish temperature-over-time data point."""
        self.publish('temperature-over-time', temp_time)

    def send_bulk(self, data: dict) -> None:
        """Publish multiple feed values at once. Keys are feed names."""
        for feed, value in data.items():
            self.publish(feed, value)

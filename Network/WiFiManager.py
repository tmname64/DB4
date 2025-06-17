import network
import time

class WiFiManager:
    def __init__(self, ssid: str, password: str, timeout: int = 20):
        self.ssid = ssid
        self.password = password
        self.timeout = timeout
        self._ap = network.WLAN(network.AP_IF)
        self._sta = network.WLAN(network.STA_IF)

    def connect(self) -> bool:
        # Disable access-point
        if self._ap.active():
            self._ap.active(False)

        # Enable station interface
        self._sta.active(True)
        self._sta.disconnect()
        self._sta.connect(self.ssid, self.password)

        print(f"Connecting to '{self.ssid}'...")
        start = time.time()
        while not self._sta.isconnected() and (time.time() - start) < self.timeout:
            print('.', end='')
            time.sleep(1)
        print()

        if self._sta.isconnected():
            ip = self._sta.ifconfig()[0]
            print(f"Connected! IP address: {ip}")
            return True
        else:
            print("Failed to connect within timeout.")
            return False

    def disconnect(self) -> None:
        if self._sta.active():
            self._sta.disconnect()
            self._sta.active(False)
            print("WiFi disconnected.")

    def is_connected(self) -> bool:
        return self._sta.isconnected()

    def ifconfig(self) -> tuple:
        """(ip, netmask, gateway, DNS)"""
        return self._sta.ifconfig()

    def scan(self) -> list:
        """(ssid, bssid, channel, RSSI, authmode, hidden)"""
        # Ensure STA interface is active for scanning
        if not self._sta.active():
            self._sta.active(True)
        return self._sta.scan()

    def enable_ap(self, ap_ssid: str = None, ap_password: str = None, channel: int = 6) -> None:
        """ --> Hotspot Mode <-- """
        # Disable station mode
        if self._sta.active():
            self._sta.active(False)

        # Configure and activate AP
        self._ap.active(True)
        if ap_ssid:
            cfg = {'essid': ap_ssid}
            if ap_password:
                cfg['authmode'] = network.AUTH_WPA_WPA2_PSK
                cfg['password'] = ap_password
            self._ap.config(**cfg)
        self._ap.config(channel=channel)
        print(f"Access-point '{ap_ssid or self._ap.config('essid')}' enabled.")




### Example usage:
# wifi = WiFiManager("Mob", "Moldova1")
# if wifi.connect():
#     print(wifi.ifconfig())
# else:
#     print("Running offline.")

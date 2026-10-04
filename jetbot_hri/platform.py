"""Read-only platform telemetry. Hardware imports are lazy."""
from collections import deque
from dataclasses import dataclass
import ipaddress
import statistics
import subprocess
import time
from typing import Optional


@dataclass
class WifiStatus:
    connected: bool = False
    ssid: Optional[str] = None
    ip_address: Optional[str] = None
    error: Optional[str] = None


def get_wifi_status():
    try:
        result = subprocess.run(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION",
                                 "device", "status"], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, universal_newlines=True, timeout=3)
        if result.returncode:
            raise RuntimeError(result.stderr.strip())
        for line in result.stdout.splitlines():
            fields = line.split(":", 3)
            if len(fields) != 4 or fields[1] != "wifi" or fields[2] != "connected":
                continue
            profile = fields[3]
            ssid_result = subprocess.run(["nmcli", "-g", "802-11-wireless.ssid",
                                          "connection", "show", profile], stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE, universal_newlines=True, timeout=3)
            ssid = ssid_result.stdout.strip() or None if ssid_result.returncode == 0 else None
            ip_result = subprocess.run(["nmcli", "-g", "IP4.ADDRESS", "device", "show", fields[0]],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       universal_newlines=True, timeout=3)
            ip_address = None
            for candidate in ip_result.stdout.splitlines():
                try:
                    ip_address = str(ipaddress.ip_interface(candidate).ip)
                    break
                except ValueError:
                    pass
            return WifiStatus(True, ssid, ip_address)
        return WifiStatus()
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        return WifiStatus(error=str(exc))


def is_wifi_connected():
    return get_wifi_status().connected


def get_ip_address():
    return get_wifi_status().ip_address


@dataclass
class BatteryReading:
    voltage: float
    current_ma: Optional[float] = None
    power_mw: Optional[float] = None
    percent: Optional[int] = None
    status: str = "SENSOR_ERROR"


class INA219Backend:
    def __init__(self, config):
        self.config = config
        self.sensor = None
        self.kind = None

    def read(self):
        cfg = self.config
        address = cfg["i2c_address"]
        if not isinstance(address, int):
            raise RuntimeError("INA219 I2C address uncalibrated; run i2cdetect")
        if self.sensor is None:
            try:
                from ina219 import INA219
                self.sensor = INA219(cfg["shunt_ohms"], busnum=cfg["i2c_bus"], address=address)
                self.sensor.configure()
                self.kind = "pi"
            except (ImportError, OSError, RuntimeError):
                from smbus2 import SMBus
                self.sensor = SMBus(cfg["i2c_bus"])
                self.kind = "smbus2"
        if self.kind == "pi":
            return BatteryReading(float(self.sensor.voltage()),
                                  float(self.sensor.current()), float(self.sensor.power()))
        raw = self.sensor.read_i2c_block_data(address, 0x02, 2)
        value = (raw[0] << 8) | raw[1]
        if value & 1:
            raise RuntimeError("INA219 bus-voltage conversion overflow")
        voltage = (value >> 3) * 0.004
        shunt = self.sensor.read_i2c_block_data(address, 0x01, 2)
        shunt_raw = (shunt[0] << 8) | shunt[1]
        if shunt_raw & 0x8000:
            shunt_raw -= 0x10000
        current_ma = (shunt_raw * 0.00001 / cfg["shunt_ohms"]) * 1000
        return BatteryReading(voltage, current_ma, voltage * current_ma)

    def close(self):
        if self.kind == "smbus2" and self.sensor is not None:
            self.sensor.close()


class MockBatteryBackend:
    def __init__(self, voltage=11.4):
        self.voltage = voltage

    def read(self):
        return BatteryReading(self.voltage)

    def close(self):
        pass


class BatteryMonitor:
    def __init__(self, backend, config, clock=time.monotonic):
        self.backend, self.config, self.clock = backend, config, clock
        self.samples = deque(maxlen=int(config["filter_window"]))
        self.low_since = None
        self.last_sample_at = None
        self.critical_latched = False
        self.last = BatteryReading(0.0)

    def read(self):
        now = self.clock()
        try:
            reading = self.backend.read()
            if not 0 < reading.voltage <= 16:
                raise ValueError("implausible battery voltage")
        except Exception:
            self.last = BatteryReading(0.0, status="SENSOR_ERROR")
            self.critical_latched = True
            return self.last
        self.samples.append(reading.voltage)
        self.last_sample_at = now
        filtered = statistics.median(self.samples)
        if len(self.samples) == self.samples.maxlen and filtered < self.config["critical_voltage"]:
            if self.low_since is None:
                self.low_since = now
            if now - self.low_since >= self.config["critical_hold_sec"]:
                self.critical_latched = True
        else:
            self.low_since = None
        reading.status = ("CRITICAL" if self.critical_latched else
                          "LOW" if filtered < self.config["warning_voltage"] else "NORMAL")
        self.last = reading
        return reading

    def is_critical_latched(self):
        return self.critical_latched

    def reset_latch(self):
        if (self.last.status == "SENSOR_ERROR" or len(self.samples) < self.samples.maxlen
                or min(self.samples) < self.config["warning_voltage"]):
            raise RuntimeError("Reset requires fresh healthy samples after charging/checking battery")
        self.critical_latched = False
        self.low_since = None
        self.last.status = "NORMAL"

    def healthy(self):
        return (not self.critical_latched and self.last.status != "SENSOR_ERROR"
                and self.last_sample_at is not None
                and self.clock() - self.last_sample_at <= self.config["max_sample_age_sec"]
                and len(self.samples) == self.samples.maxlen)

    def close(self):
        self.backend.close()

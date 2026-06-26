"""
CONEX-PSD9 Beam Position Tracker
Reads X/Y laser beam position from a Newport CONEX-PSD9 over USB serial.

Serial settings: 921600 baud, 8N1, no flow control (USB, preset by device).
Command format:  <address><CMD>\r\n  (e.g. "1GP\r\n")
Response format: <address><CMD><values>  (e.g. "1GP3.125,-2.962,52")
"""

import serial
import serial.tools.list_ports
import time
import sys


BAUD_RATE    = 921600
TIMEOUT      = 1.0      # seconds to wait for a response
CONTROLLER   = 1        # default controller address (1-31)
POLL_HZ      = 10       # position reads per second


class ConexPSD:
    def __init__(self, port: str, address: int = CONTROLLER):
        self.address = address
        self.ser = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        time.sleep(0.1)  # allow USB enumeration to settle
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser.is_open:
            self.ser.close()

    # ------------------------------------------------------------------
    # Low-level I/O
    # ------------------------------------------------------------------
    def _send(self, cmd: str) -> str:
        """Send a command and return the stripped response line."""
        packet = f"{self.address}{cmd}\r\n".encode("ascii")
        self.ser.write(packet)
        raw = self.ser.readline()
        return raw.decode("ascii").strip()

    def _parse_response(self, response: str, cmd: str) -> str:
        """Strip the address+command prefix and return the payload."""
        prefix = f"{self.address}{cmd}"
        if response.startswith(prefix):
            return response[len(prefix):]
        return response  # unexpected format — return as-is

    # ------------------------------------------------------------------
    # Device queries
    # ------------------------------------------------------------------
    def get_firmware(self) -> str:
        resp = self._send("VE")
        return self._parse_response(resp, "VE").strip()

    def get_state(self) -> str:
        """Return raw state string, e.g. '000032' (32 = READY, 14 = CONFIG)."""
        resp = self._send("TS")
        return self._parse_response(resp, "TS")

    def check_error(self) -> str:
        """Return the last error code ('@' means no error)."""
        resp = self._send("TE")
        return self._parse_response(resp, "TE")

    def get_position(self) -> tuple[float, float, float]:
        """
        Return (x_mm, y_mm, laser_power_pct) from the GP command.
        Raises ValueError if the response cannot be parsed.
        """
        resp = self._send("GP")
        payload = self._parse_response(resp, "GP")
        parts = payload.split(",")
        if len(parts) != 3:
            raise ValueError(f"Unexpected GP response: {resp!r}")
        x, y, lp = float(parts[0]), float(parts[1]), float(parts[2])
        return x, y, lp

    def get_raw_inputs(self) -> tuple[float, float, float]:
        """Return raw ADC (X, Y, SUM) for the silicon PSD9 sensor."""
        resp = self._send("RA")
        payload = self._parse_response(resp, "RA")
        parts = payload.split(",")
        return tuple(float(p) for p in parts)

    def get_corrected_inputs(self) -> tuple[float, float, float]:
        """Return offset/gain-corrected ADC (X, Y, SUM)."""
        resp = self._send("RC")
        payload = self._parse_response(resp, "RC")
        parts = payload.split(",")
        return tuple(float(p) for p in parts)

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------
    def is_ready(self) -> bool:
        state = self.get_state()
        return state.endswith("32")


# ------------------------------------------------------------------
# Helper: list available serial ports
# ------------------------------------------------------------------
def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for p in ports:
        print(f"  {p.device:10s} — {p.description}")

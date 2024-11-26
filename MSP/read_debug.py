import serial
import struct

class MSPClient:
    MSP_DEBUG = 254  # MSP_DEBUG identifier (example, may vary)
    DEBUG16_VALUE_COUNT = 8  # Number of debug values to process

    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(port, baudrate, timeout=2)

    def construct_msp_request(self, code):
        """Construct an MSP request."""
        header = b'$M<'  # MSP header
        payload = b''  # No payload for the request
        size = len(payload)
        checksum = size ^ code
        return header + bytes([size, code, checksum])

    def send_request(self, code):
        """Send an MSP request and get the response."""
        request = self.construct_msp_request(code)
        self.serial.write(request)
        return self.read_response()

    def read_response(self):
        """Read the MSP response."""
        # Wait for the MSP header
        header = self.serial.read(3)
        if header != b'$M>':
            raise ValueError("Invalid header received")

        # Read payload size and code
        size = self.serial.read(1)[0]
        code = self.serial.read(1)[0]

        # Read payload and checksum
        payload = self.serial.read(size)
        checksum = self.serial.read(1)[0]

        # Validate checksum
        calculated_checksum = size ^ code
        for byte in payload:
            calculated_checksum ^= byte

        if calculated_checksum != checksum:
            raise ValueError("Checksum mismatch")

        return code, payload

    def get_debug_values(self):
        """Request and parse MSP_DEBUG values."""
        code, payload = self.send_request(self.MSP_DEBUG)
        if len(payload) != self.DEBUG16_VALUE_COUNT * 2:
            raise ValueError("Unexpected payload size")

        # Unpack 16-bit debug values
        debug_values = struct.unpack(f"<{self.DEBUG16_VALUE_COUNT}H", payload)
        return debug_values

    def close(self):
        """Close the serial connection."""
        if self.serial.is_open:
            self.serial.close()

# Example usage:
if __name__ == "__main__":
    port = '/dev/ttyACM0'  # Replace with your flight controller's port
    client = MSPClient(port)

    try:
        debug_values = client.get_debug_values()
        print("Debug values:", debug_values)
    finally:
        client.close()

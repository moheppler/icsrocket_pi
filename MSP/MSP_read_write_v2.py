import serial
import time
import struct
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import threading

# MSP Command Constants
MSP_SET_RAW_RC = 200  # Set RC Channels
MSP_RAW_IMU = 102  # Get RAW IMU Data
MSP_DEBUG = 254  # Command to get debug data


# Serial Port Settings
SERIAL_PORT = "/dev/ttyACM0"  # Update based on your setup
BAUD_RATE = 115200

# Initialize RC Channels (4 Channels: Roll, Pitch, Throttle, Yaw)
rc_channels = [1500, 1500, 1500, 1500]
send_rc = True

# Serial Communication Setup
def send_msp_command(serial_conn, cmd, payload=[]):
    """Send an MSP command with optional payload."""
    size = len(payload)
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b

    packet = (
        b"$M<"  # Header
        + bytes([size])  # Payload size
        + bytes([cmd])  # Command
        + bytes(payload)  # Payload
        + bytes([checksum])  # Checksum
    )
    serial_conn.write(packet)


def read_msp_response(serial_conn):
    """Read and parse MSP response."""
    header = serial_conn.read(3)  # Read header
    if header != b"$M>":
        return None

    size = serial_conn.read(1)[0]  # Payload size
    cmd = serial_conn.read(1)[0]  # Command
    payload = serial_conn.read(size)  # Payload
    checksum = serial_conn.read(1)[0]  # Checksum

    # Verify checksum
    computed_checksum = size ^ cmd
    for b in payload:
        computed_checksum ^= b
    if computed_checksum != checksum:
        return None

    return cmd, payload


def poll_imu_data(serial_conn):
    """Constantly poll IMU data and debug values."""
    while True:
        # Poll IMU data
        send_msp_command(serial_conn, MSP_RAW_IMU)
        response = read_msp_response(serial_conn)
        if response and response[0] == MSP_RAW_IMU:
            imu_data = struct.unpack("<hhhhhhhhh", response[1])
            print(f"IMU Data: {imu_data}")

        # Poll debug values
        send_msp_command(serial_conn, MSP_DEBUG)
        response = read_msp_response(serial_conn)
        if response and response[0] == MSP_DEBUG:
            debug_values = struct.unpack("<hhhhhhhh", response[1])
            print(f"Debug Values: {debug_values}")

        time.sleep(0.1)  # Poll every 100ms



def update_rc_channels(serial_conn):
    """Send updated RC channel values."""
    while send_rc:
        payload = []
        for ch in rc_channels:
            payload += list(struct.pack("<H", ch))  # Convert to little-endian 16-bit
        print(f"Setting RC Channels: {rc_channels}")
        send_msp_command(serial_conn, MSP_SET_RAW_RC, payload)
        time.sleep(0.1)  # Send every 100ms


# Dash App for Control
app = dash.Dash(__name__)

app.layout = html.Div([
    html.H1("RC Channel Control"),
    html.Div([
        html.Label("Roll"),
        dcc.Slider(1000, 2000, 1, value=1500, id="roll-slider"),
        html.Label("Pitch"),
        dcc.Slider(1000, 2000, 1, value=1500, id="pitch-slider"),
        html.Label("Throttle"),
        dcc.Slider(1000, 2000, 1, value=1500, id="throttle-slider"),
        html.Label("Yaw"),
        dcc.Slider(1000, 2000, 1, value=1500, id="yaw-slider"),
    ])
])


@app.callback(
    Output("roll-slider", "value"),
    Output("pitch-slider", "value"),
    Output("throttle-slider", "value"),
    Output("yaw-slider", "value"),
    [Input("roll-slider", "value"),
     Input("pitch-slider", "value"),
     Input("throttle-slider", "value"),
     Input("yaw-slider", "value")]
)
def update_channels(roll, pitch, throttle, yaw):
    """Update RC channel values from sliders."""
    rc_channels[0] = roll
    rc_channels[1] = pitch
    rc_channels[2] = throttle
    rc_channels[3] = yaw
    print(f"updating rc_channels: {rc_channels}")
    return roll, pitch, throttle, yaw


if __name__ == "__main__":
    # Open Serial Connection
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        exit()

    # Start IMU Polling in a Separate Thread
    imu_thread = threading.Thread(target=poll_imu_data, args=(serial_conn,))
    imu_thread.daemon = True
    imu_thread.start()
    
    # Start RC Channel Update in a Separate Thread
    rc_thread = threading.Thread(target=update_rc_channels, args=(serial_conn,))
    rc_thread.daemon = True
    rc_thread.start()

    # Start Dash Server
    try:
        app.run_server(debug=True, host="0.0.0.0", use_reloader=False)
    finally:
        send_rc = False
        rc_thread.join()
        #   Close serial connection on exit
        serial_conn.close()

import serial
import struct
import time
import threading
from dash import Dash, dcc, html
from dash.dependencies import Input, Output
from multiprocessing import Value

# MSP Command IDs
MSP_SET_RAW_RC = 200  # Command to set RC channels
MSP_RAW_IMU = 102     # Command for RAW IMU data (9 DOF)

# Initialize the Dash app
app = Dash(__name__)

# Use multiprocessing.Value to store slider values safely across threads
slider_value1 = Value('d', 1500)  # Aileron (centered at 1500)
slider_value2 = Value('d', 1500)  # Elevator (centered at 1500)
slider_value3 = Value('d', 1500)  # Rudder (centered at 1500)
slider_value4 = Value('d', 1500)  # Throttle (centered at 1500)

# Create a Lock to ensure thread-safe access to the global values
lock = threading.Lock()

# Function to update the global values based on slider inputs
def process_values(ser):
    """Simulate a loop that processes the slider values and sends them as RC commands."""
    while True:
        with lock:
            # Read the values from the shared memory
            val1 = slider_value1.value  # Aileron
            val2 = slider_value2.value  # Elevator
            val3 = slider_value3.value  # Rudder
            val4 = slider_value4.value  # Throttle
            print(f"Updated channel values: {val1}, {val2}, {val3}, {val4}")

            # Prepare the RC channels list
            channels = [int(val1), int(val2), int(val3), int(val4), 1500, 1500, 1500, 1500]

            # Send the MSP_SET_RAW_RC command to set RC values
            set_raw_rc(ser, channels)
        
        time.sleep(0.1)  # Simulate processing delay
        # print("in process values")


# Function to send the MSP_SET_RAW_RC command
def set_raw_rc(ser, channels):
    """Set the raw RC channels. Channels should be a list of 8 integers."""
    assert len(channels) == 8, "There should be exactly 8 channels"
    payload = []
    for ch in channels:
        payload += [ch & 0xFF, (ch >> 8) & 0xFF]  # Convert to little-endian 16-bit values
    print(f"Setting RC Channels: {channels}")  # Debug: Print the channel values
    send_msp_command(ser, MSP_SET_RAW_RC, payload)
    
    
# MSP Command functions
def calculate_checksum(payload):
    """Calculate MSP checksum (XOR of all payload bytes)."""
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return checksum

def build_msp_command(command, payload=[]):
    """Build an MSP command frame to send."""
    header = b'$M<'  # MSP header
    size = len(payload)
    checksum = calculate_checksum([size, command] + payload)
    frame = header + bytes([size, command] + payload + [checksum])
    return frame

def send_msp_command(ser, command, payload=[]):
    frame = build_msp_command(command, payload)
    # with lock:
    ser.write(frame)
    ser.flush()

    # Debug: Print what was sent
    print(f"Sent frame: {frame.hex()}")
    
    # Read response safely
    try:
        response_header = ser.read(3)
        if len(response_header) < 3:
            print("Incomplete response header")
            return None
        size = ser.read(1)[0]
        command = ser.read(1)[0]
        response_payload = ser.read(size)
        checksum = ser.read(1)[0]
        return response_payload
    except Exception as e:
        print(f"Error during serial communication: {e}")
        return None


def parse_msp_raw_imu(payload):
    """Parse the MSP_RAW_IMU payload (message ID 102)."""
    if len(payload) < 18:
        raise ValueError("Invalid payload size for MSP_RAW_IMU")

    # Unpack the raw IMU data (9 degrees of freedom)
    acc_x, acc_y, acc_z = struct.unpack('<hhh', payload[0:6])  # Acceleration in m/s^2 (scaled)
    gyro_x, gyro_y, gyro_z = struct.unpack('<hhh', payload[6:12])  # Gyroscope in degrees/s (scaled)
    mag_x, mag_y, mag_z = struct.unpack('<hhh', payload[12:18])  # Magnetometer in microtesla (scaled)

    return {
        "acc_x": acc_x,
        "acc_y": acc_y,
        "acc_z": acc_z,
        "gyro_x": gyro_x,
        "gyro_y": gyro_y,
        "gyro_z": gyro_z,
        "mag_x": mag_x,
        "mag_y": mag_y,
        "mag_z": mag_z,
    }

def poll_and_print_imu(ser):
    """Continuously poll IMU data from the flight controller and print it."""
    while True:
        try:
            # Send the MSP_RAW_IMU command (no payload required)
            imu_payload = send_msp_command(ser, MSP_RAW_IMU)

            # Parse the IMU data
            imu_data = parse_msp_raw_imu(imu_payload)

            # Print the parsed IMU data
            # print(f"IMU Data: {imu_data}")
        
        except Exception as e:
            print(f"Error reading IMU data: {e}")
        
        time.sleep(0.1)  # Polling interval

def start_processing_loop(ser):
    """Start the RC processing loop in a separate thread."""
    processing_thread = threading.Thread(target=process_values, args=(ser,))
    processing_thread.daemon = True
    processing_thread.start()

def start_imu_polling_loop(ser):
    """Start the IMU polling loop in a separate thread."""
    imu_thread = threading.Thread(target=poll_and_print_imu, args=(ser,))
    imu_thread.daemon = True
    imu_thread.start()

# Define the layout of the Dash app
app.layout = html.Div([
    html.H1("Interactive RC Control Sliders"),
    
    dcc.Slider(
        id='slider1',
        min=1000,
        max=2000,
        step=1,
        value=1500,
        marks={i: str(i) for i in range(1000, 2100, 100)},
        tooltip={"placement": "bottom", "always_visible": True},
    ),
    
    dcc.Slider(
        id='slider2',
        min=1000,
        max=2000,
        step=1,
        value=1500,
        marks={i: str(i) for i in range(1000, 2100, 100)},
        tooltip={"placement": "bottom", "always_visible": True},
    ),
    
    dcc.Slider(
        id='slider3',
        min=1000,
        max=2000,
        step=1,
        value=1500,
        marks={i: str(i) for i in range(1000, 2100, 100)},
        tooltip={"placement": "bottom", "always_visible": True},
    ),
    
    dcc.Slider(
        id='slider4',
        min=1000,
        max=2000,
        step=1,
        value=1500,
        marks={i: str(i) for i in range(1000, 2100, 100)},
        tooltip={"placement": "bottom", "always_visible": True},
    ),
])

# Define the callback functions to update the global values
@app.callback(
    [Output('slider1', 'value'),
     Output('slider2', 'value'),
     Output('slider3', 'value'),
     Output('slider4', 'value')],
    [Input('slider1', 'value'),
     Input('slider2', 'value'),
     Input('slider3', 'value'),
     Input('slider4', 'value')]
)
def update_slider_values(val1, val2, val3, val4):
    global slider_value1, slider_value2, slider_value3, slider_value4

    # Acquire lock before modifying global variables to ensure thread-safe access
    with lock:
        # Update shared values
        slider_value1.value = val1
        slider_value2.value = val2
        slider_value3.value = val3
        slider_value4.value = val4
    print(f"Updated slider values: {slider_value1.value}, {slider_value2.value}, {slider_value3.value}, {slider_value4.value}")

    
    # Return the same values to the sliders (this is a simple "echo" back)
    return slider_value1.value, slider_value2.value, slider_value3.value, slider_value4.value

if __name__ == '__main__':
    # Open the serial connection to the flight controller (adjust as needed)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    
    # Start the RC command processing loop
    start_processing_loop(ser)
    
    # Start the IMU polling loop
    start_imu_polling_loop(ser)
    
    # Start the Dash server
    app.run_server(debug=False, host='0.0.0.0', port=8050)

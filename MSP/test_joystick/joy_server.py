import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import threading
import time
from multiprocessing import Value

# Initialize the Dash app
app = dash.Dash(__name__)

# Use multiprocessing.Value to store slider values safely across threads
slider_value1 = Value('d', 0)  # 'd' for double (float) type
slider_value2 = Value('d', 0)
slider_value3 = Value('d', 0)
slider_value4 = Value('d', 0)

# Create a Lock to ensure thread-safe access to the global values
lock = threading.Lock()

# Function to update the global values based on slider inputs
def process_values():
    """Simulate a loop that processes the slider values."""
    while True:
        current_thread = threading.current_thread()  # Get the current thread
        print(f"Thread Name: {current_thread.name}, Thread ID: {current_thread.ident}")
        with lock:
            # Read the values from the shared memory
            val1 = slider_value1.value
            val2 = slider_value2.value
            val3 = slider_value3.value
            val4 = slider_value4.value

            # Process the values (this is just a placeholder for actual logic)
            print(f"Processing values: {val1}, {val2}, {val3}, {val4}")

        time.sleep(0.1)  # Simulate processing delay

# Start the processing loop in a separate thread
def start_processing_loop():
    processing_thread = threading.Thread(target=process_values)
    processing_thread.daemon = True  # Daemonize the thread to terminate with the main app
    processing_thread.start()

# Define the layout of the Dash app
app.layout = html.Div([
    html.H1("Interactive Sliders"),
    
    dcc.Slider(
        id='slider1',
        min=0,
        max=100,
        step=1,
        value=0,
        marks={i: str(i) for i in range(0, 101, 10)},
        tooltip={"placement": "bottom", "always_visible": True},
    ),
    
    dcc.Slider(
        id='slider2',
        min=0,
        max=100,
        step=1,
        value=0,
        marks={i: str(i) for i in range(0, 101, 10)},
        tooltip={"placement": "bottom", "always_visible": True},
    ),
    
    dcc.Slider(
        id='slider3',
        min=0,
        max=100,
        step=1,
        value=0,
        marks={i: str(i) for i in range(0, 101, 10)},
        tooltip={"placement": "bottom", "always_visible": True},
    ),
    
    dcc.Slider(
        id='slider4',
        min=0,
        max=100,
        step=1,
        value=0,
        marks={i: str(i) for i in range(0, 101, 10)},
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
    
    # Return the same values to the sliders (this is a simple "echo" back)
    return slider_value1.value, slider_value2.value, slider_value3.value, slider_value4.value

if __name__ == '__main__':
    start_processing_loop()  # Start the processing loop in a separate thread
    app.run_server(debug=False, host='0.0.0.0', port=8050)

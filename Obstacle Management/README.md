# **Obstacle Management**

## Description

This code serves as the central control system for managing obstacles and making navigational decisions. It processes data from a camera mounted on the vehicle, assesses the surroundings, and sends commands to control the vehicle's movements. Additionally, it offers a web-based interface for monitoring and debugging the decision-making process.

### How It Works

1. **Camera Setup**: The code initializes and configures a camera on the vehicle to capture images of the environment. The vehicle's velocity has been deliberately kept low to ensure better image processing and accuracy in obstacle detection.

2. **Image Processing**: It analyzes the images from the camera to identify critical elements such as obstacles, traffic signals, and road features by examining their colors and patterns.

3. **Decision-Making**: Based on the information extracted from the images, the code makes informed decisions about how the vehicle should navigate. For instance, it can recognize a red traffic signal and command the vehicle to stop or detect obstacles to adjust its path accordingly.

4. **Sending Commands**: The code translates these decision commands into motor and steering control instructions, thus dictating the vehicle's speed and direction to navigate obstacles effectively.

5. **Debugging and Monitoring**: The code provides a user-friendly interface for visualizing its decision-making process, making it easier to debug and verify the vehicle's navigation choices. This web-based monitoring is active during the debugging phase. It establishes a web-based monitoring interface that enables users to track the vehicle's decision-making process through a web page, facilitating testing and ensuring the vehicle operates correctly.

6. **Real-Time Processing**: The code continuously processes incoming images and makes real-time decisions, emulating the way a human driver continually assesses their surroundings and adapts to changing conditions.

In essence, this code functions as the "brain" and "eyes" of the vehicle, enabling it to perceive its surroundings, make informed navigation decisions, and execute those decisions with precision.

## Pseudocode

```python
# Pseudocode for Obstacle Management

# Initialize the camera, current image, and necessary variables
Initialize camera with specified parameters
Create an empty 2D array currentImage to store the current frame
Initialize lc_time and present_time variables to keep track of time
Create a Queue called turnSet with a maximum size of 10 and fill it with 0s

# Define functions for image processing and decision-making
Function split_image(current_frame):
    # Image processing to identify objects
    # ...
End Function

Function create_blob():
    # Create blobs to detect objects
    # ...
End Function

Function get_dir(wallDifferences2, wallHeightsRaw):
    # Determine rotation direction
    # ...
End Function

Function wall_split(wall_vimage):
    # Split the wall image
    # ...
End Function

Function get_wall_turn(walls, heights, cl):
    # Calculate wall-based turn
    # ...
End Function

Function get_traffic_turn(redp, greenp):
    # Calculate traffic signal-based turn
    # ...
End Function

Function calculate(current_frame):
    # Main processing function
    # ...
End Function

# Define a function to start the camera
Function camera_start():
    Set camera.running to True
    
    Define a function cap() to capture frames continuously:
        Capture the current frame from the camera
        Sleep for a short duration to control the frame capture rate
        
    Create a thread cam_th to run the cap() function
    Start the cam_th thread

# Define a function to read the current frame
Function read():
    Rotate the currentImage by 180 degrees
    Return the rotated frame

# Define a function to run the control system in debug mode
Function run_debug():
    Initialize lc_time and present_time to keep track of time
    
    Infinite loop:
        Read the current frame
        Split the frame into wall_line, green_mask, and red_mask
        Calculate the turn and debug_frame using the calculate() function
        
        Update the turnSet queue with the calculated turn value
        
        If the queue is not full:
            Put the rounded turn value in the queue
        
        If the queue is not empty:
            Map the turn value to a range of [0, 110] and send it to a serial port for control
        
        Update lc_time to the current time
        
        Try:
            Encode the debug_frame as a JPEG image and yield it as a multipart response
        Except for any exceptions, print the error message

# Define a function to run the control system
Function run():
    Infinite loop:
        Read the current frame
        Split the frame into wall_line, green_mask, and red_mask
        Calculate the turn using the calculate() function
        
        Map the turn value to a range of [0, 110] for control
        
        Send the mapped turn value to a serial port for control
        Print the turn value

# Define the main Flask application
Create a Flask app called "app"

Define a route '/' that returns a multipart response with frames from the run_debug() function

If the script is run as the main program:
    Initialize a serial port connection
    Start the camera using camera_start()
    Run the Flask app on host '0.0.0.0' and port 5000
```

The flowchart representing the logic and workflow of the code can be found below:

![Flowchart](https://github.com/zero-abd/baby_musks_wroBot/assets/48104263/fc672cc6-d66e-45d6-82f6-31062deab6df)

import camera_code as cc
import subprocess
import cv2
import time
import requests
import base64
import numpy as np
API_URL = "http://localhost:5000/predict"


def classify_image(image_path):
    """Send the image to the API and display the image received in the response."""
    with open(image_path, 'rb') as f:
        response = requests.post(API_URL, files={'image': f})

    # Parse the JSON response
    data = response.json()

    # If the response contains an image in base64 format
    if 'image' in data:
        img_data = base64.b64decode(data['image'])

        # Convert the base64 byte data to an image using OpenCV
        np_arr = np.frombuffer(img_data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image
        if img is not None:
            cv2.imshow('API Image', img)
            cv2.waitKey(2000)  # Display for 3 seconds
            cv2.destroyAllWindows()
        else:
            print("Failed to decode image.")
    else:
        print("No image found in the response.")
def exitListen():
    # Replace the placeholder with the actual command
    script = "bash -c 'source /fsm_piston/devel/setup.bash && rosrun demo demo_leave_listen_node'"

    # Execute the command using subprocess
    subprocess.run(script, shell=True)


def isListenActive():
    # Run the rosrun command and capture the output
    result = subprocess.run(
        ['rosrun', 'demo', 'demo_ask_sta'],   # Command to run
        stdout=subprocess.PIPE,               # Capture the standard output
        stderr=subprocess.PIPE,               # Capture the error output
        text=True                             # Return output as string (Python 3.7+)
    )
    # Check if the output contains the phrase 'subdata is true'
    if "subdata is true" in result.stdout:
        print("Abc")
        return True
    else:
        return False


def display_image(image_path, display_time=800):
    # Load the image with OpenCV
    img = cv2.imread(image_path)

    if img is not None:
        # Display the image
        # Naming a window 
        cv2.namedWindow("Captured Image", cv2.WINDOW_NORMAL) 
        
        # Using resizeWindow() 
        cv2.resizeWindow("Captured Image", 1980,1080) 
        cv2.imshow('Captured Image', img)
        # Wait for display_time ms (3 seconds) or until a key is pressed
        cv2.waitKey(display_time)
        # Close the image window
        cv2.destroyAllWindows()
    else:
        print(f"Could not load image from {image_path}")


flag = 0
switch = True
counter = 1

subprocess.run("bash -c 'source /fsm_piston/devel/setup.bash'", shell=True)

while switch:
    try:
        b = isListenActive()
    except:
        b = False

    if b is True: 
        print(flag)
        if flag == 0:
            image_path = cc.click_image(flag, counter)
            classify_image(image_path)
            flag = 1
            print("Hooray")

            # Display the captured image for 3 seconds
            display_image(image_path)

            # First camera call goes here
            try:
                if isListenActive():
                    exitListen()
            except:
                continue

        elif flag >= 1:
            image_path = cc.click_image(flag, counter)
            flag += 1

            # Display the captured image for 3 seconds
            display_image(image_path)

            if flag == 16:
                flag = 0
                counter += 1

            print("Hurrah again")

            # Second camera call
            try:
                if isListenActive():
                    exitListen()
            except:
                continue

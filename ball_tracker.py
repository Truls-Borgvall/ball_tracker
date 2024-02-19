"""
    File: ball_tracker.py
    Author: Truls Borgvall
    Date: 2024-02-19
    Description: This program finds the position and velocity of the ball using picamera and openCV. It then displays the position, velocity and required vector to move the ball to the center on the screen.
"""

# Import all the required libraries
import cv2 as cv
import numpy as np
from picamera2 import Picamera2

"""
    Creates a binary mask of the frame with an upper range and lower range. The color of the ball becomes white(on) and the rest becomes black(off).
    Parameters:
    - frame: The current frame from the camera.
    Returns: The binary mask.
"""
def create_mask(frame):
    # Convert frame to HSV color space
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Define lower and upper bounds for the color of the ball in HSV space. In this case for a white ball with a black background.
    lower_range = np.array([0, 0, 150])
    upper_range = np.array([180, 100, 255])

    # Create a mask using the specified color range
    mask = cv.inRange(hsv, lower_range, upper_range)
    return mask

"""
    Finds the contours from the mask and the finds the biggest calculates the biggest contour.
    Parameters:
    - mask: The current binary mask.
    Returns: The ball contour if a ball contour is detected. Otherwise, None.
"""
def find_ball_contour(mask):
    # Find contours in the mask
    contours, hierarchies = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if contours:
        # Get the contour with maximum area (the ball)
        ball_contour = max(contours, key=cv.contourArea)
        return ball_contour
    else:
        return None

"""
    Finds the centroid of the ball contour.
    Parameters:
    - ball_contour: The current ball_contour from the find_ball_contour function.
    Returns: The x value and y value of the centroid if a balls area is not zero. If the area is zero it returns None.
"""
def find_ball_pos(ball_contour):
    # Calculate moments of the contour to find its centroid
    M = cv.moments(ball_contour)

    if M["m00"] != 0:
        # Calculate centroid coordinates (x, y)
        cx = int(M["m10"] / M["m00"]) # Centroid x-coordinate
        cy = int(M["m01"] / M["m00"]) # Centroid y-coordinate
        return cx, cy
    else:
        return None

"""
    Calculates the velocity of the ball in the x and y direction by using the formula V = s/t. s is the distance and t is the time.
    Parameters:
    - ball_pos: The x and y cordinates of the centroid from the ball contour.
    - prev_pos: The previous x and y cordinates of the centroid from the ball contour.
    - fps: The fps of the camera. The fps value is hardcoded.
    Returns:
    If the ball has a previos position and it has a current position the velocity in the x and y value is returend. Else None is returned.
"""
def calculate_ball_velocity(ball_pos, prev_pos, fps):
    if prev_pos is not None and ball_pos is not None:
        # Calculate velocity of the ball based on its current and previous positions
        velocity_x = (ball_pos[0] - prev_pos[0]) / (1/fps)
        velocity_y = (ball_pos[1] - prev_pos[1]) / (1/fps)
        
        return velocity_x, velocity_y
    else:
        return None

"""
    Draws a circle around the ball, draws an arrow from the postion of the ball indicating the relative size of the balls velocity, draws an arrow indicating the required vector that is required to move the ball to the center. Finally the balls positon and speed is written on the screen along with information about what the colors indicate.
    Parameters:
    - frame: The current frame from the camera.
    - ball_pos: The current x and y values of the balls position.
    - ball_velocity: The current x and y values of the balls velocity
    Returns:
    None
"""
def draw(frame, ball_pos, ball_velocity):
    if ball_pos and ball_velocity:
         # Draw a circle at the current position of the ball
        frame = cv.circle(frame, (ball_pos[0], ball_pos[1]), 10, (0,255,0), 1)

        # Calculate the end point of the arrow using the velocity
        end_point = (int(ball_pos[0] + ball_velocity[0]), int(ball_pos[1] + ball_velocity[1]))
        
        # Draw the velocity vector as an arrow
        frame = cv.arrowedLine(frame, ball_pos, end_point, (255, 0, 0), 1)
        # Draw a line indicating the required vector for the ball to move towards the center of the frame
        frame = cv.arrowedLine(frame, end_point, (frame.shape[1]//2, frame.shape[0]//2),  (0,0,255), 1)
        
        # Display textual information about ball position and velocity on the frame
        cv.putText(frame, f"Green: Ball position: {ball_pos}", (10, frame.shape[0]-90), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv.putText(frame, f"Blue: Ball speed: {ball_velocity} px/s", (10, frame.shape[0]-50), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv.putText(frame, "Red: Ball's required vector to go to the center", (10, frame.shape[0]-10), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

"""
    The main function of the program. Runs all the functions required to calculate the balls position, speed, and required vector to move the ball to the center. Finally it draws it on the screen.
    Parameters:
    None
    Returns:
    None
"""
def main():
    # Initialize the Picamera2 object
    piCam = Picamera2()
    
    # Set resolution for the camera preview
    width = 1280
    height = 720
    piCam.preview_configuration.main.size = (width,height)
    
    # Configure preview format to BGR for OpenCV compatibility
    piCam.preview_configuration.main.format = "RGB888"
    
    # Set camera framerate
    piCam.framerate = 30
    fps = 30
    
    # Align preview configuration
    piCam.preview_configuration.align()
    piCam.configure("preview")
    piCam.start()


    prev_pos = None

    while True:
        # Capture frame from the camera
        frame = piCam.capture_array()
        
        # Create mask to detect the ball
        mask = create_mask(frame)
        # Find contour of the ball
        ball_contour = find_ball_contour(mask)

        # Find position of the ball
        ball_pos = find_ball_pos(ball_contour)
        # Calculate velocity of the ball
        ball_velocity = calculate_ball_velocity(ball_pos, prev_pos, fps)
        prev_pos = ball_pos

        # Draw ball position and velocity on the frame
        draw(frame, ball_pos, ball_velocity)
        
        # Display the frame
        cv.imshow("frame", frame)
        
        # Break the loop if "q" is pressed
        if cv.waitKey(1) & 0xFF == ord("q"): break

    # Release resources
    cv.destroyAllWindows()


main()
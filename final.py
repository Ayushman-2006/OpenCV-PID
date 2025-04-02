import cv2
import numpy as np
import math
import socket
def main():
    cap = cv2.VideoCapture("https://192.168.242.217:8080/video")
        
    while True:
        ret, image = cap.read()
        if not ret:
            print("Failed to receive frame")
            break

        # Process the image
        processed_binary, processed_contours = process(test(image))

        # Resize the images
        width, height = 600, 600
        resized_binary = cv2.resize(processed_binary, (width, height), interpolation=cv2.INTER_AREA)
        resized_contours = cv2.resize(processed_contours, (width, height), interpolation=cv2.INTER_AREA)

        bot_info = get_bot_position(resized_contours)

        if bot_info:
            center_x, center_y = map(int, bot_info["position"])
            orientation = bot_info["orientation"]
            
            # Draw the bot position and direction
            cv2.circle(resized_contours, (center_x, center_y), 10, (0, 0, 255), -1)  # Red dot at bot position
            arrow_length = 50
            end_x = int(center_x + arrow_length * np.cos(np.radians(orientation)))
            end_y = int(center_y + arrow_length * np.sin(np.radians(orientation)))
            cv2.arrowedLine(resized_contours, (center_x, center_y), (end_x, end_y), (255, 0, 0), 3)
            cv2.putText(resized_contours, f"Angle: {orientation:.2f}", (center_x + 20, center_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Continue processing that depends on bot position
            a, r = c(resized_binary)
            print(a)
            cv2.circle(resized_contours, (a[0], a[1]), 10, (0, 0, 255), -1)
            t = calculate_target_point((center_x, center_y), 230,a)
            error=calculate_cross_track_error(a,t,(center_x,center_y))
            print("error: ",error)
            cv2.circle(resized_contours, (int(t[0]), int(t[1])), 10, (0, 255, 0), -1)
            print("bot coord: ",(center_x,center_y),"angle: ",orientation,"target: ", t)
            
            udp(error)
            
        else:
            print("No bot detected")
            # Optionally, you can skip further processing that depends on bot position
            # Or set default values for center_x and center_y if needed.
            a, r = c(resized_binary)
            print(a)
            cv2.circle(resized_contours, (a[0], a[1]), 10, (0, 0, 255), -1)
            # Skipping calculate_circular_target and path_tracking_controller since bot position is unknown.
         
        # Display the processed images
        cv2.imshow("Binary", resized_binary)
        cv2.imshow("Contours", resized_contours)

        # Wait for a key press to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

def udp(error):
    # ESP8266 IP and Port
    ESP8266_IP = "192.168.114.138"  
    ESP8266_PORT = 12345           

    # Create a UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_command(error):
        """Send a command to the ESP8266 via UDP"""
        message = f"{error}"
        udp_socket.sendto(message.encode(), (ESP8266_IP, ESP8266_PORT))
        print(f"Sent: {message}")
    
    send_command(error) 
    udp_socket.close()  # Close the socket after sending
   

    udp_socket.close()  # Close the socket after sending
def process(frame):
    # Convert to grayscale and apply Gaussian blur
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # Reduce blur to keep sharp edges

    # Enhance contrast
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # Find contours using RETR_EXTERNAL
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours
    contour_frame = frame.copy()
    cv2.drawContours(contour_frame, contours, -1, (0, 255, 0), 2)

    return binary, contour_frame

def test(frame):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    if ids is None or len(ids) < 4:
        print("Not enough markers detected")
        return frame
    else:
        print("yes")
    
    marker_corners = {}
    for i in range(len(ids)):
        marker_id = ids[i][0]
        marker_corners[marker_id] = corners[i][0]

    src = np.float32([
        marker_corners[1][0],  # Top-left marker, top-left corner
        marker_corners[2][1],  # Top-right marker, top-right corner
        marker_corners[3][2],  # Bottom-right marker, bottom-right corner
        marker_corners[4][3]   # Bottom-left marker, bottom-left corner
    ])
    dst = np.float32([
        [0, 0],       # Top-left
        [499, 0],     # Top-right
        [499, 499],   # Bottom-right
        [0, 499]      # Bottom-left
    ])
    m = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(frame, m, (500, 500))
    return warped


def initialize_aruco_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    parameters = cv2.aruco.DetectorParameters()
    return dictionary, parameters

def detect_markers(frame, dictionary, parameters):
    corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    return corners, ids

def calculate_angle(point1, point2):
    delta_x = point2[0] - point1[0]
    delta_y = point2[1] - point1[1]
    return np.arctan2(delta_y, delta_x) * 180 / np.pi

def get_bot_position(frame):
    dictionary, parameters = initialize_aruco_detector()
    corners, ids = detect_markers(frame, dictionary, parameters)
    
    if ids is None or len(corners) == 0:
        return None
    
    marker_corners = corners[0][0]  # First detected marker's corners
    center_x = np.mean(marker_corners[:, 0])
    center_y = np.mean(marker_corners[:, 1])
    
    front_mid = np.mean(marker_corners[0:2], axis=0)  # Front edge midpoint
    back_mid = np.mean(marker_corners[2:4], axis=0)  # Back edge midpoint
    orientation = calculate_angle(back_mid, front_mid)
    
    return {
        "position": (center_x, center_y),
        "orientation": orientation
    }


import math
def c(frame):
   
    contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea)
    if len(largest_contour)>0:
        # Fit a circle to the contour
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        center = (int(x), int(y))
        radius = int(radius)
    
  

    return center,radius



####################################################################################################################
import math

def calculate_target_point(center, radius, bot_position):
    """
    Calculate target point on circle's circumference
    
    Args:
    - center: Circle center coordinates
    - radius: Circle radius
    - bot_position: Current bot position
    - lookahead_distance: Distance along arc to look ahead
    
    Returns:
    - Target point coordinates
    """
    # Vector from center to bot
    dx = bot_position[0] - center[0]
    dy = bot_position[1] - center[1]
    
    # Current angle of bot relative to center
    current_angle = math.atan2(dy, dx)
    
    # Calculate target angle
    
    target_angle = current_angle #+ angle_increment
    
    # Target point on circumference
    target_x = center[0] + radius * math.cos(target_angle)
    target_y = center[1] + radius * math.sin(target_angle)
    
    return (target_x, target_y)

def calculate_cross_track_error(bot_position, target_point, center):
    """
    Calculate cross-track error with signed magnitude
    
    Args:
    - bot_position: Current bot position
    - target_point: Calculated target point
    - center: Circle center
    
    Returns:
    - Signed error value
    """
    # Vector calculations
    dx = target_point[0] - bot_position[0]
    dy = target_point[1] - bot_position[1]
    
    # Cross product to determine side
    # cross_product = (target_point[0] - center[0]) * (bot_position[1] - center[1]) /(target_point[1] - center[1]) * (bot_position[0] - center[0])
    
    # Calculate distance
    distance = math.sqrt(dx**2 + dy**2)
    
    # Sign the error based on which side of the line the bot is on
    return -distance 

if __name__ == "__main__":
    main()
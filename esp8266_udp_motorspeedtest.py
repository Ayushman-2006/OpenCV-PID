import socket

# ESP8266 IP and Port
ESP8266_IP = "192.168.242.138"  
ESP8266_PORT = 12345           

# Create a UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(motor, direction, speed):
    """Send a command to the ESP8266 via UDP"""
    message = f"{motor},{direction},{speed}"
    udp_socket.sendto(message.encode(), (ESP8266_IP, ESP8266_PORT))
    print(f"Sent: {message}")


send_command(2,'f',10) 
send_command(1,'f',10) 




 


# udp_socket.close()  # Close the socket after sending

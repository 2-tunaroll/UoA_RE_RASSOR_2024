import pygame
import socket
import json

# Initialise pygame
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Set up the socket connection
HOST = "ENTER IP ADDRESS OF PI HERE"
PORT = 8000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Attempting to connect to Raspberry Pi...")
sock.connect((HOST, PORT))
print(f"Connected to Raspberry Pi at {HOST}:{PORT}")

def send_command(data):
    # gets commands and sends them over the socket connection
    message = json.dumps(data)
    sock.sendall((message + "\n").encode('utf-8'))

try:
    print("Sending controller state to Raspberry Pi...")
    while True:
        # get controller inputs
        pygame.event.pump()
        axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

        # Construct a dictionary of the controller state
        controller_state = {
            'axes': axes,
            'buttons': buttons
        }
        # Send the controller state to the Raspberry Pi
        send_command(controller_state)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    sock.close()
    pygame.quit()
    print("Connection closed and pygame quit.")
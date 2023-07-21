import pygame
import time
import socket
import struct
import math


from NatNetClient import NatNetClient
from util import quaternion_to_euler

positions = {}
rotations = {}

def receive_rigid_body_frame(id, position, rotation_quaternion):
    # Position and rotation received
    positions[id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    # Store the roll pitch and yaw angles
    rotations[id] = (rotx, roty, rotz)

# udp params
UDP_IP = "192.168.0.48"
#UDP_IP = "192.168.0.81"
UDP_PORT = 1234
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

def udp_init():
    sock = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM
    ) # UDP
    return sock

def joystick_init():
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

    # Prints the values for axis0
    joystick = pygame.joystick.Joystick(0)
    return joystick

def init():
    sock = udp_init()
    joystick = joystick_init()
    return sock, joystick

def udp_send(sock, ip, port, message):
    sock.sendto(message, (UDP_IP, UDP_PORT))


if __name__ == "__main__":
    sock, joystick = init()
    l = 0.2 # meters
    absz = 0
    b_old = 0
    b_state = 1
    a_old = 0
    a_state = 1
    clientAddress = "192.168.0.107"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 371

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    x_pid = PID(0.1, 0, 0, setpoint = 2)
    y_pid = PID(0.1, 0, 0, setpoint = 0)
    z_pid = PID(3, 0, 0, setpoint = 2, sample_time=0.01)


    while True:
        time_start = time.time()
        # Get the joystick readings
        pygame.event.pump()
        b = joystick.get_button(1)
        a = joystick.get_button(0)
        if a == 1 and a_old == 0:
            a_state = not a_state
        a_old = a
        if b == 1 and b_old == 0:
            b_state = not b_state
        b_old = b

        fx = x_pid(positions[robot_id][0])
        fy = y_pid(positions[robot_id][1])
        fz = z_pid(positions[robot_id][2])

        # fx = -joystick.get_axis(1) # left handler: up-down, inverted
        # fy = joystick.get_axis(0) # left handler: left-right
        # fz = -joystick.get_axis(4)  # right handler: up-down, inverted
        tauz = 0 
        taux = 0
        tauy = joystick.get_axis(3) # right handler: left-right
        #absz = .5
        if abs(joystick.get_axis(4)) > .1:
            absz += -0.02*joystick.get_axis(4)
        if absz < 0 or b_state == 0:
            absz = 0
            
        #OptiTrack
        if (is_running & robot_id in positions):
        # last position
            Last_position = positions[robot_id]
            rotation = rotations[robot_id]
            tauz = rotation[2]
        
        print(round(fx,2), round(fy,2), round(fz/8,2), round(a_state,2), round(tauy,2), tauz, round(absz,2), round(b_state, 2))
        message = struct.pack('<ffffffff', fx, fy , fz/8, a_state, tauy, tauz, absz, b_state) 
        udp_send(sock, UDP_IP, UDP_PORT, message)
        #print(message)
        while(time.time() - time_start < 0.02):
            time.sleep(0.02)

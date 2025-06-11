#!/usr/bin/env python3
import socket
import time

def send_tcp_command(host, port, command):
    """
Send TCP command to the specified device and receive response
Parameters:
        host: Target device IP address
        port: Destination Port
        command: The command string to send
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5.0)
            

            s.connect((host, port))
            print(f"✅ Successfully connected to {host}:{port}")
            
            s.sendall(command.encode() + b'\n')  
            print(f"📤 Command sent: {command}")
            
            response = s.recv(4096)
            print(f"📥 Receive response ({len(response)} byte):")
            print(response.decode(errors='replace'))
            
    except Exception as e:
        print(f"❌ An error occurred: {str(e)}")

if __name__ == "__main__":
    # Configuration parameters (modify according to actual situation)
    TARGET_IP = "192.168.1.100"   # Device IP address
    TARGET_PORT = 2401           # Device Port
    COMMAND1 = "TEST_MEMS_stream_onoff:1"        
    COMMAND2 = "TEST_MAIN_mems_offset_set:7910"
    COMMAND3 = "TEST_MOTOR_motor_control:1"    

    send_tcp_command(TARGET_IP, TARGET_PORT, COMMAND1)
    time.sleep(1)
    send_tcp_command(TARGET_IP, TARGET_PORT, COMMAND2)
    time.sleep(1)  
    send_tcp_command(TARGET_IP, TARGET_PORT, COMMAND3)

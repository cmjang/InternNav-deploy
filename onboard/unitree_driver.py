#!/usr/bin/env python3
"""
File: unitree_driver_10hz.py
Role: Unitree Go2 Driver optimized for low-frequency UDP inputs (10Hz).
Logic:
  1. Priority: Physical Remote Controller (RC) > UDP Script.
  2. If RC joystick moves > deadzone: Script yields control immediately.
  3. If RC is idle: Script checks UDP buffer for new commands.
"""
import sys
import time
import socket
import struct

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient

# === IMPORTANT: Imports for Go2 Robot ===
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

# === Configuration ===
LOCAL_IP = "127.0.0.1"
PORT = 8899
UDP_TIMEOUT = 1.0       # Increased timeout to accommodate 10Hz (0.1s) input
REMOTE_DEADZONE = 0.1   # Physical joystick deadzone
SCRIPT_DEADZONE = 0.02  # UDP command deadzone

# === Remote Controller Parser Class ===
class UnitreeRemoteController:
    def __init__(self):
        self.Lx = 0.0
        self.Rx = 0.0
        self.Ry = 0.0
        self.Ly = 0.0

    def parse(self, raw_data):
        """
        Efficiently parses raw bytes from LowState wireless_remote.
        Format: [Head...] [Lx, Rx, Ry, L2, Ly] [...Tail]
        """
        try:
            # Convert to bytes and unpack 5 floats starting at offset 4
            # Layout: Lx(4), Rx(8), Ry(12), L2(16, unused), Ly(20)
            buf = bytes(raw_data)
            data = struct.unpack_from('<5f', buf, 4)
            
            self.Lx = data[0]
            self.Rx = data[1]
            self.Ry = data[2]
            # data[3] is L2 (unused)
            self.Ly = data[4]
        except Exception:
            pass # Ignore parsing errors (e.g., startup noise)

    def is_active(self):
        """Returns True if any stick is moved beyond the deadzone."""
        return (abs(self.Lx) > REMOTE_DEADZONE or 
                abs(self.Rx) > REMOTE_DEADZONE or 
                abs(self.Ry) > REMOTE_DEADZONE or 
                abs(self.Ly) > REMOTE_DEADZONE)

# === Low State Subscriber Class ===
class RobotStateMonitor:
    def __init__(self):
        self.remote = UnitreeRemoteController()
        self.subscriber = None

    def init(self):
        # Subscribe to lowstate to get remote controller data
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.handler, 10)
    
    def handler(self, msg: LowState_):
        # Update remote controller state from incoming message
        self.remote.parse(msg.wireless_remote)

# === Main Driver ===
def main():
    print("=== Starting Unitree Driver (10Hz Optimized) ===")
    
    # 1. Initialize SDK
    try:
        if len(sys.argv) > 1:
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0)
    except Exception as e:
        print(f"[Error] SDK Init failed: {e}")
        sys.exit(1)

    # 2. Setup Robot State Monitor (Remote Override)
    monitor = RobotStateMonitor()
    monitor.init()
    print("[State] Monitoring Remote Controller...")

    # 3. Setup Sport Client (Control)
    sport = SportClient()
    sport.SetTimeout(10.0)
    sport.Init()
    
    # Optional: Initial wake up
    try:
        sport.BalanceStand()
        print("[Robot] Robot control initialized.")
    except:
        pass

    # 4. Setup UDP Server
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LOCAL_IP, PORT))
    sock.setblocking(False) # Non-blocking mode is crucial

    print(f"[Network] UDP Listening on {PORT}")
    print("[System] Priority Mode: REMOTE > UDP SCRIPT")

    # Variables
    udp_vx, udp_vyaw = 0.0, 0.0
    last_udp_time = time.time()
    script_is_controlling = False

    try:
        while True:
            # --- A. Priority Check: Physical Remote ---
            if monitor.remote.is_active():
                if script_is_controlling:
                    print("[Interrupt] Remote moved! Releasing control.")
                    script_is_controlling = False
                
                # If remote is active, sleep briefly and skip this loop
                # This ensures the physical remote has full control
                time.sleep(0.05) 
                continue

            # --- B. UDP Data Handling (Drain Buffer) ---
            got_packet = False
            try:
                # Read ALL available packets in the buffer to get the latest one
                # This prevents lag if the script runs slower than the sender
                while True:
                    data, _ = sock.recvfrom(1024)
                    udp_vx, udp_vyaw = struct.unpack('ff', data)
                    got_packet = True
            except BlockingIOError:
                pass # No new data
            except Exception:
                pass 

            if got_packet:
                last_udp_time = time.time()

            # --- C. Safety Watchdog ---
            # Reset velocity if no data received for UDP_TIMEOUT
            if time.time() - last_udp_time > UDP_TIMEOUT:
                udp_vx, udp_vyaw = 0.0, 0.0

            # --- D. Script Control Logic ---
            has_udp_input = abs(udp_vx) > SCRIPT_DEADZONE or abs(udp_vyaw) > SCRIPT_DEADZONE

            if has_udp_input:
                if not script_is_controlling:
                    print("[Control] Script active.")
                    script_is_controlling = True
                
                # Send velocity command
                sport.Move(udp_vx, 0.0, udp_vyaw)

            else:
                # If script was active but input is now zero, stop safely
                if script_is_controlling:
                    print("[Control] Stopping.")
                    sport.Move(0.0, 0.0, 0.0)
                    script_is_controlling = False
                
                # If idle, do nothing (pass)
                pass

            # Loop Rate: ~25Hz (0.04s)
            # Sufficient for 10Hz UDP inputs while keeping CPU usage low
            time.sleep(0.04)

    except KeyboardInterrupt:
        print("\n[Exit] Stopping...")
        sport.StopMove()

if __name__ == '__main__':
    main()
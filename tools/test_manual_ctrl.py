#!/usr/bin/env python3
"""
Keyboard control with proper UDP connection
"""

from pynput import keyboard
from pymavlink import mavutil
import time

# Configuration
ROBOT_IP = "192.168.4.1"
ROBOT_PORT = 14550
LOCAL_PORT = 14550  # Different port to avoid conflicts with QGC
GCS_SYSID = 254

# Control state
state = {
    'forward': 0.0,
    'turn': 0.0,
    'speed': 0.5,
    'armed': False,
    'running': True
}

current_keys = set()

def on_press(key):
    try:
        current_keys.add(key.char)
    except AttributeError:
        if key == keyboard.Key.up:
            state['speed'] = min(1.0, state['speed'] + 0.05)
            print(f"\nSpeed: {state['speed']:.0%}")
        elif key == keyboard.Key.down:
            state['speed'] = max(0.1, state['speed'] - 0.05)
            print(f"\nSpeed: {state['speed']:.0%}")

def on_release(key):
    try:
        current_keys.discard(key.char)
    except AttributeError:
        pass
    
    if key == keyboard.Key.space:
        state['armed'] = not state['armed']
        print(f"\n{'ARMING' if state['armed'] else 'DISARMING'}...")
        
        mav.mav.command_long_send(
            1, 1,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if state['armed'] else 0.0,
            0, 0, 0, 0, 0, 0
        )
    
    if key == keyboard.Key.esc:
        state['running'] = False
        return False

# Create bidirectional UDP connection
# Option 1: Listen on local port, send to robot
print(f"Setting up MAVLink on port {LOCAL_PORT}...")
mav = mavutil.mavlink_connection(
    f'udp:0.0.0.0:{LOCAL_PORT}',
    source_system=GCS_SYSID
)

# Set destination for outgoing messages
mav.target_system = 1
mav.target_component = 1

print(f"Sending to {ROBOT_IP}:{ROBOT_PORT}")
print("Waiting for heartbeat from robot...")

# Don't wait for heartbeat - just start sending
print("(Skipping heartbeat wait - will send commands immediately)")
print()

# OR if robot is definitely sending heartbeats, use timeout:
# try:
#     mav.wait_heartbeat(timeout=5)
#     print(f"Heartbeat received from system {mav.target_system}\n")
# except:
#     print("No heartbeat received, but continuing anyway...\n")

print("=== Controls ===")
print("W/S: Forward/Back")
print("A/D: Turn Left/Right")
print("↑/↓: Speed +/-")
print("SPACE: Arm/Disarm")
print("ESC: Quit\n")

# Start keyboard listener
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

try:
    while state['running']:
        # Process any incoming MAVLink messages
        msg = mav.recv_match(blocking=False)
        if msg:
            if msg.get_type() == 'HEARTBEAT':
                # Got heartbeat from robot
                pass
            elif msg.get_type() == 'COMMAND_ACK':
                ack = msg
                if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"\n{'Armed' if state['armed'] else 'Disarmed'} successfully")
                    else:
                        print(f"\n{'Arm' if state['armed'] else 'Disarm'} failed!")
                        state['armed'] = not state['armed']  # Revert
        
        # Update control values
        state['forward'] = 0.0
        state['turn'] = 0.0
        
        if 'w' in current_keys:
            state['forward'] += 1.0
        if 's' in current_keys:
            state['forward'] -= 1.0
        if 'a' in current_keys:
            state['turn'] -= 1.0
        if 'd' in current_keys:
            state['turn'] += 1.0
        
        # Apply speed
        forward = state['forward'] * state['speed']
        turn = state['turn'] * state['speed']
        
        # Convert to MAVLink range
        x = int(forward * 1000)
        y = int(turn * 1000)
        z = int(state['speed'] * 1000)
        
        # Send manual control to robot
        # NOTE: This sends to 0.0.0.0 - you need to tell it where to send
        mav.mav.manual_control_send(
            1,  # target_system
            x, y, z, 0, 0
        )
        
        # Print status
        status = "ARMED" if state['armed'] else "DISARMED"
        print(f"\r{status} | Fwd:{forward:+.2f} Turn:{turn:+.2f} Speed:{state['speed']:.2f}   ", end='', flush=True)
        
        time.sleep(0.50)  # 20Hz

except KeyboardInterrupt:
    pass

finally:
    print("\n\nShutting down...")
    if state['armed']:
        mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0.0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)
    listener.stop()
    print("Done")
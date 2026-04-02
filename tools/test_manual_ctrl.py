#!/usr/bin/env python3
"""
Keyboard and joystick control with proper UDP connection
"""

from pynput import keyboard
from pymavlink import mavutil
import time
import threading

# Try SDL2 for gamepad support
SDL2_AVAILABLE = False
try:
    import sdl2
    SDL2_AVAILABLE = True
    print("Using SDL2 for gamepad support")
except ImportError:
    print("SDL2 not installed - joystick support disabled")
    print("Install with: pip install pysdl2 pysdl2-dll")

# Configuration
ROBOT_IP = "192.168.4.1"
ROBOT_PORT = 14550
LOCAL_PORT = 14550
GCS_SYSID = 254

# Joystick configuration
JOYSTICK_DEADZONE = 0.1
AXIS_MAX = 32767.0
SPEED_STEP = 0.05

# Button indices (0-based) - adjust if needed for your controller
BUTTON_SPEED_DOWN = 1  # Button 7 (0-indexed)
BUTTON_SPEED_UP = 2    # Button 8 (0-indexed)
BUTTON_ARM = 0        # Cross/A button

# Control state
state = {
    'forward': 0.0,
    'turn': 0.0,
    'speed': 0.5,
    'armed': False,
    'running': True,
    'input_source': 'keyboard',
    'joy_forward': 0.0,
    'joy_turn': 0.0,
    'joy_btn_arm': False,
    'joy_btn_speed_up': False,
    'joy_btn_speed_down': False
}

current_keys = set()
controller = None
joystick = None

def init_joystick():
    global controller, joystick
    
    if not SDL2_AVAILABLE:
        return False
    
    sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_GAMECONTROLLER)
    
    num_joysticks = sdl2.SDL_NumJoysticks()
    if num_joysticks == 0:
        print("No joystick detected - using keyboard only")
        return False
    
    # Skip GameController API - use raw Joystick API for direct button control
    # This gives us consistent button indices we can configure
    joystick = sdl2.SDL_JoystickOpen(0)
    if joystick:
        name = sdl2.SDL_JoystickName(joystick)
        axes = sdl2.SDL_JoystickNumAxes(joystick)
        buttons = sdl2.SDL_JoystickNumButtons(joystick)
        print(f"Joystick connected: {name.decode()}")
        print(f"  Axes: {axes}, Buttons: {buttons}")
        return True
    
    print("Failed to open joystick")
    return False


def sdl2_thread():
    """Background thread to read SDL2 gamepad/joystick"""
    while state['running']:
        try:
            # Pump SDL events
            sdl2.SDL_PumpEvents()
            
            if controller:
                # GameController API - use standard button constants
                # Left stick Y for forward/reverse
                ly = sdl2.SDL_GameControllerGetAxis(controller, sdl2.SDL_CONTROLLER_AXIS_LEFTY)
                state['joy_forward'] = -ly / AXIS_MAX  # Invert Y
                
                # Right stick X for turning
                rx = sdl2.SDL_GameControllerGetAxis(controller, sdl2.SDL_CONTROLLER_AXIS_RIGHTX)
                state['joy_turn'] = rx / AXIS_MAX
                
                # Use GameController button constants (standardized mapping)
                state['joy_btn_arm'] = sdl2.SDL_GameControllerGetButton(
                    controller, sdl2.SDL_CONTROLLER_BUTTON_A) == 1  # Cross on PS
                state['joy_btn_speed_down'] = sdl2.SDL_GameControllerGetButton(
                    controller, sdl2.SDL_CONTROLLER_BUTTON_LEFTSHOULDER) == 1  # L1
                state['joy_btn_speed_up'] = sdl2.SDL_GameControllerGetButton(
                    controller, sdl2.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) == 1  # R1
                
            elif joystick:
                # Raw Joystick API - use raw button indices
                # Left stick Y (axis 1) for forward/reverse
                if sdl2.SDL_JoystickNumAxes(joystick) >= 2:
                    ly = sdl2.SDL_JoystickGetAxis(joystick, 1)
                    state['joy_forward'] = -ly / AXIS_MAX  # Invert Y
                
                # Right stick X (axis 2) for turning
                if sdl2.SDL_JoystickNumAxes(joystick) >= 3:
                    rx = sdl2.SDL_JoystickGetAxis(joystick, 2)
                    state['joy_turn'] = rx / AXIS_MAX
                
                # Buttons - use raw indices
                num_buttons = sdl2.SDL_JoystickNumButtons(joystick)
                if BUTTON_ARM < num_buttons:
                    state['joy_btn_arm'] = sdl2.SDL_JoystickGetButton(joystick, BUTTON_ARM) == 1
                if BUTTON_SPEED_DOWN < num_buttons:
                    state['joy_btn_speed_down'] = sdl2.SDL_JoystickGetButton(joystick, BUTTON_SPEED_DOWN) == 1
                if BUTTON_SPEED_UP < num_buttons:
                    state['joy_btn_speed_up'] = sdl2.SDL_JoystickGetButton(joystick, BUTTON_SPEED_UP) == 1
            
        except Exception as e:
            print(f"\nSDL2 error: {e}")
        
        time.sleep(0.02)

def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)

def get_joystick_input():
    if not controller and not joystick:
        return 0.0, 0.0, False, False, False
    
    forward = apply_deadzone(state['joy_forward'], JOYSTICK_DEADZONE)
    turn = apply_deadzone(state['joy_turn'], JOYSTICK_DEADZONE)
    
    return forward, turn, state['joy_btn_arm'], state['joy_btn_speed_up'], state['joy_btn_speed_down']

def on_press(key):
    try:
        current_keys.add(key.char)
    except AttributeError:
        if key == keyboard.Key.up:
            state['speed'] = min(1.0, state['speed'] + SPEED_STEP)
            print(f"\nSpeed: {state['speed']:.0%}")
        elif key == keyboard.Key.down:
            state['speed'] = max(0.1, state['speed'] - SPEED_STEP)
            print(f"\nSpeed: {state['speed']:.0%}")

def on_release(key):
    try:
        current_keys.discard(key.char)
    except AttributeError:
        pass
    
    if key == keyboard.Key.space:
        toggle_arm()
    
    if key == keyboard.Key.esc:
        state['running'] = False
        return False

def toggle_arm():
    state['armed'] = not state['armed']
    print(f"\n{'ARMING' if state['armed'] else 'DISARMING'}...")
    
    mav.mav.command_long_send(
        1, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if state['armed'] else 0.0,
        0, 0, 0, 0, 0, 0
    )

# Initialize joystick
has_joystick = init_joystick()

# Start joystick thread if available
if has_joystick:
    joy_thread = threading.Thread(target=sdl2_thread, daemon=True)
    joy_thread.start()

# Create bidirectional UDP connection
print(f"Setting up MAVLink on port {LOCAL_PORT}...")
mav = mavutil.mavlink_connection(
    f'udp:0.0.0.0:{LOCAL_PORT}',
    source_system=GCS_SYSID
)

mav.target_system = 1
mav.target_component = 1

print(f"Sending to {ROBOT_IP}:{ROBOT_PORT}")
print("(Skipping heartbeat wait - will send commands immediately)")
print()

print("=== Controls ===")
print("Keyboard:")
print("  W/S: Forward/Back")
print("  A/D: Turn Left/Right")
print("  Arrow Up/Down: Speed +/-")
print("  SPACE: Arm/Disarm")
print("  ESC: Quit")
if has_joystick:
    print("Joystick:")
    print("  Left Stick Y: Forward/Reverse")
    print("  Right Stick X: Turn")
    print("  Button 7: Speed down")
    print("  Button 8: Speed up")
    print("  Cross/A Button: Arm/Disarm toggle")
print()

# Start keyboard listener
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Track button states for edge detection
joy_btn_arm_prev = False
joy_btn_speed_up_prev = False
joy_btn_speed_down_prev = False

try:
    while state['running']:
        # Process any incoming MAVLink messages
        msg = mav.recv_match(blocking=False)
        if msg:
            if msg.get_type() == 'HEARTBEAT':
                pass
            elif msg.get_type() == 'COMMAND_ACK':
                ack = msg
                if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"\n{'Armed' if state['armed'] else 'Disarmed'} successfully")
                    else:
                        print(f"\n{'Arm' if state['armed'] else 'Disarm'} failed!")
                        state['armed'] = not state['armed']
        
        # Get joystick input
        joy_forward, joy_turn, joy_btn_arm, joy_btn_speed_up, joy_btn_speed_down = 0.0, 0.0, False, False, False
        if has_joystick:
            joy_forward, joy_turn, joy_btn_arm, joy_btn_speed_up, joy_btn_speed_down = get_joystick_input()
            
            # Edge detection for arm button
            if joy_btn_arm and not joy_btn_arm_prev:
                toggle_arm()
            joy_btn_arm_prev = joy_btn_arm
            
            # Edge detection for speed buttons
            if joy_btn_speed_up and not joy_btn_speed_up_prev:
                state['speed'] = min(1.0, state['speed'] + SPEED_STEP)
                print(f"\nSpeed: {state['speed']:.0%}")
            joy_btn_speed_up_prev = joy_btn_speed_up
            
            if joy_btn_speed_down and not joy_btn_speed_down_prev:
                state['speed'] = max(0.1, state['speed'] - SPEED_STEP)
                print(f"\nSpeed: {state['speed']:.0%}")
            joy_btn_speed_down_prev = joy_btn_speed_down
        
        # Get keyboard input
        kb_forward = 0.0
        kb_turn = 0.0
        
        if 'w' in current_keys:
            kb_forward += 1.0
        if 's' in current_keys:
            kb_forward -= 1.0
        if 'a' in current_keys:
            kb_turn -= 1.0
        if 'd' in current_keys:
            kb_turn += 1.0
        
        # Combine inputs - joystick takes priority if active
        if abs(joy_forward) > 0.01 or abs(joy_turn) > 0.01:
            state['forward'] = joy_forward
            state['turn'] = joy_turn
            state['input_source'] = 'joystick'
        else:
            state['forward'] = kb_forward
            state['turn'] = kb_turn
            if kb_forward != 0 or kb_turn != 0:
                state['input_source'] = 'keyboard'
        
        # Apply speed
        forward = state['forward'] * state['speed']
        turn = state['turn'] * state['speed']
        
        # Convert to MAVLink range
        x = int(forward * 1000)
        y = int(turn * 1000)
        z = int(state['speed'] * 1000)
        
        # Send manual control to robot
        mav.mav.manual_control_send(
            1,
            x, y, z, 0, 0
        )
        
        # Print status
        status = "ARMED" if state['armed'] else "DISARMED"
        src = f"[{state['input_source'][:3].upper()}]"
        print(f"\r{status} {src} | Fwd:{forward:+.2f} Turn:{turn:+.2f} Speed:{state['speed']:.2f}   ", end='', flush=True)
        
        time.sleep(0.05)  # 20Hz

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
        time.sleep(0.05)
    listener.stop()
    
    # Cleanup SDL2
    if SDL2_AVAILABLE:
        if controller:
            sdl2.SDL_GameControllerClose(controller)
        if joystick:
            sdl2.SDL_JoystickClose(joystick)
        sdl2.SDL_Quit()
    
    print("Done")
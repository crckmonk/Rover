import socket
import json
import time
import sys

try:
    import keyboard
except ImportError:
    print("\nThe 'keyboard' module is required for interactive control.")
    print("Install it with: python -m pip install keyboard")
    sys.exit(1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 14550))

esp_address = ('192.168.4.1', 14550)

base_speed = 50
left_speed = 0
right_speed = 0
direction = 0

print("Controls: arrow keys to drive, Shift/Control to tweak base speed (default 50).")
print("  Up/down set direction FWD/REV at base speed.")
print("  Left/right adjust one motor = opposite - 30.")
print("  Hold Shift to increase base speed, Ctrl to decrease.")
print("  Press ESC to exit.\n")

last_send = None

def send_state():
    global last_send
    msg = {"direction": direction, "left": left_speed, "right": right_speed}
    txt = json.dumps(msg)
    if txt != last_send:
        sock.sendto(txt.encode('utf-8'), esp_address)
        print(f"Sent: {txt}")
        last_send = txt

try:
    while True:
        if keyboard.is_pressed('shift'):
            base_speed = min(100, base_speed + 5)
            print(f"Base speed now {base_speed}")
            time.sleep(0.1)
        if keyboard.is_pressed('ctrl'):
            base_speed = max(0, base_speed - 5)
            print(f"Base speed now {base_speed}")
            time.sleep(0.1)

        if keyboard.is_pressed('up'):
            direction = 1
            left_speed = base_speed
            right_speed = base_speed
            send_state()
        elif keyboard.is_pressed('down'):
            direction = 2
            left_speed = base_speed
            right_speed = base_speed
            send_state()
        elif keyboard.is_pressed('left'):
            left_speed = max(0, right_speed - 30)
            send_state()
        elif keyboard.is_pressed('right'):
            right_speed = max(0, left_speed - 30)
            send_state()
        else:
            if direction != 0 or left_speed != 0 or right_speed != 0:
                direction = 0
                left_speed = 0
                right_speed = 0
                send_state()

        if keyboard.is_pressed('esc'):
            print("Exiting")
            break

        time.sleep(0.02)
finally:
    sock.close()

import sdl2
import time

sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)
js = sdl2.SDL_JoystickOpen(0)

print("Press buttons to see their indices...")
while True:
    sdl2.SDL_PumpEvents()
    for i in range(sdl2.SDL_JoystickNumButtons(js)):
        if sdl2.SDL_JoystickGetButton(js, i):
            print(f"Button {i} pressed")
    time.sleep(0.1)
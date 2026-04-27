 # STM32 based remotely controlled rover with MavLink support
## Requirements: 

### Basic hardware requirements 
 - 4 Wheel skid steering chassis
 - Powered by a swappable, UPS type 12V battery.
   
### Control 

 - 2 Remote control channels: 
    1. Primary control via MavLink over UDP
    2. Fallback manual control over LoRa
 - MavLink common messages support
 - Manual control via MavLink MANUAL_CONTROL protocol
 - Simple custom manual control protocol to be used over a fallback comms channel
   
### Telemetry
 - Battery voltage monitoring. Battery state estimation
 - Heading calculation based on magnetometer and accelerometer data
 - Wheel odometry calculated by reading motor encoders 

### Misc
 - Serial shell configuration/monitoring interface
 - FPV video transmission 
 - Onboard peripherals controlled remotely(Lights, camera mount turning, ...)


## Current state 03-04-26
### Build
![side2](https://github.com/user-attachments/assets/63a235fa-4e0e-424e-9b1c-ac35c375f921)
![side](https://github.com/user-attachments/assets/e34e4f61-7b5a-4284-9476-45f25883fe8b)
![top](https://github.com/user-attachments/assets/81eccc77-ab90-43d7-bd74-477b32d0c8c0)


### Capabilities
 - Manual control over UDP via MavLink manual control messages. Can be configured to A. Launch a Soft AP B. Connect to existing network in station mode.
 - Heartbeat transmission, Arm/Disarm toggle. Can be controlled with MAVLink MANUAL_CONTROL messages. Supports param request/read/set to establish connection with QGC. 
 - SYS_STAT telemetry transmission.
 - Simple first person video transmission.
 - Battery voltage monitoring
 - Potential backup NRF24 control channel that requires specialized remote https://github.com/crckmonk/RoverControlV2 (Latest version is not tested, needs to be properly integrated, abandoned for now)

### Problems
 - Lacks fallback control channel.
 - 
 - Limited MavLink support.
 - ~Clumsy motor control~ 
 - ~Primitive incoming data processing algorithm in ESP driver.~

### Priority tasks
 - [x] Improve motor control (Simple tank type steering; Ensure robust control signals)
 - [x] Add support  MavLink messages required to interact with QGC
 - [] Add VTX & camera ON/OFF switching capability (Hardware and relevant MAVLink message support
 

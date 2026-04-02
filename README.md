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
<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/1db315db-4fba-4ee6-adb2-d2c59f2d25b4" />
<img width="675" height="786" alt="image" src="https://github.com/user-attachments/assets/0cdfe4c4-4d2a-4c6e-b0d3-d8a9153d40a5" />
### Capabilities
 - Manual control over UDP via MavLink manual control messages. Can be configured to A. Launch a Soft AP B. Connect to existing network in station mode.
 - Heartbeat transmission, Arm/Disarm toggle. QGC detects rover but doesn't send any manual control messages. Controlled using a python script at the moment.
 - SYS_STAT telemetry transmission.
 - Simple first person video transmission.
 - Battery voltage monitoring
 - Potential backup NRF24 control channel that requires specialized remote https://github.com/crckmonk/RoverControlV2 (Latest version is not tested, needs to be properly integrated, abandoned for now)

### Problems
 - High latency in station mode
 - Clumsy motor control
 - Primitive incoming data processing algorithm in ESP driver.
 - Very limited MavLink support.

### Priority tasks
 - Improve motor control
 - Add support for more MavLink messages (Specifically messages required to interact with QGC)

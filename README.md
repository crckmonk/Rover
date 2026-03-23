# STM32 based remotely controlled rover with MavLink support
## Requirements: 
### Basic hardware requirements 
 - 4 Wheel differential steering chassis
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


# Proustite firmware
This repository contains the Arduino files flashed to the ESP32/Arduino and the PlatformIO code for the Nucleo G431KB with a custom shield.

The repository houses code for my robot that was built for the DeltaX 2026 advanced robot football competition (which I won).
Veni, vidi, vici.

The custom Nucleo shield is not by me and there aren't any source files that I could find.
I had to manually probe all the connectors to find where they lead.

The Nucleo controls 3 DC motors with omni-wheels attached. The Nucleo does all the math needed to control the robot.

You might want to replace the shield as the drone ESC connector has an isolator between it and the Nucleo, so I couldn't probe the correct pin and the PWM signals didn't go through.

## Serial Commands
Both the Nucleo and ESP32 use serial over USB to communicate with the main Raspberry Pi computer. The baud rate is 115200.

### Nucleo commands
```javascript
Format: COMMAND[,param1,param2,...]
VEL,vx,vy,omega - Set velocity (m/s, m/s, rad/s)
STOP - Stop all motors
STATUS - Get robot status
PID,kp,ki,kd - Update PID gains
```

### ESP32/Arduino commands
```javascript
F - forward
R - reverse
S - stop
```
Do note, that the forward and reverse are arbitrary. It depends on how you mount the motor and wire it up and stuff.

## Pinout
If you really wanted to you could wire up the ESC and MPU6050 directly to the Raspberry Pi, but then some hats you may place on the Pi don't have passthrough.

The motor driver boards are custom [Robotont](https://robotont.ut.ee/en/technical-documentation/) hardware. I do not like the micro-match connectors on those, but they will do.

I would recommend swithing out the Nucleo and the shield for newer [Robotont](https://github.com/robotont/robotont-hardware-x-2023-replication-package) hardware. The shield's pinout can be found [here](https://github.com/robotont/robotont-firmware/tree/accdbe4c774cda8ef2a9e7ea0e95ad34a170a936/include). Do note that the mbed project used in the Robotont firmware has been [sunset by ARM](https://os.mbed.com/blog/entry/Important-Update-on-Mbed/).

### Nucleo shield pins
```cpp
// Motor 1
#define MOTOR1_PWM D9
#define MOTOR1_DIR1 D8
#define MOTOR1_DIR2 D7
#define MOTOR1_ENC_A D11
#define MOTOR1_ENC_B D12

// Motor 2
#define MOTOR2_PWM D0
#define MOTOR2_DIR1 A3
#define MOTOR2_DIR2 A4
#define MOTOR2_ENC_A A1
#define MOTOR2_ENC_B A0

// Motor 3
#define MOTOR3_PWM D1
#define MOTOR3_DIR1 D3
#define MOTOR3_DIR2 A6
#define MOTOR3_ENC_A D4
#define MOTOR3_ENC_B D6
```

You might need to switch the motor cables or the definitions around depending where the motors are and where are they connected.

### ESP32/Arduino

The ESP32 decided mid competition that it has had enough and decided to break off it's own micro USB connector.

How delightful!

So I have replaced it with an Arduino Micro. I haven't tested the config, but it should in theory work. 

If you want to, you can replace the ESP32 or if you end up with my robot in your hands, the Arduino should already be flashed.

I used the Arduino IDE to flash the code.

**The ESP32 doesn't flash, if you have the I2C pins connected!**

**Disconnect the MPU6050 when flashing!**

```cpp
// ESP32 pin definitions
// --- MPU6050 ---
#define SDA_PIN 2
#define SCL_PIN 15

// --- ESC ---
const int ESC_PIN = 13;
```

```cpp
// The I2C SDA and SCL can't be changed on Arduino Micro
// Pin 2 is SDA and 3 is SCL
// --- ESC ---
const int ESC_PIN = 5;
```
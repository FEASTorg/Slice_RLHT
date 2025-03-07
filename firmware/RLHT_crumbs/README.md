# RLHT Firmware Docs

Notes on the firmware for relay heater slice.

## Commands

This system supports several types of serial commands that allow the user to control various aspects of the system, such as mode selection, relay input, heater setpoints, PID tuning, and emergency stop.

### 1. Mode Commands (`M`)

- **Format**: `M <mode>`
  - **Example**: `M 0` to switch to CONTROL mode (automatic PID control)
  - **Example**: `M 1` to switch to WRITE mode (manual control of relay inputs)

### 2. Write Relay Input (`W`)

- **Format**: `W <relay> <value>`
  - **Example**: `W 1 75` to set relay 1 input to 75% (value is clamped between 0 and 100)
  - **Example**: `W 2 50` to set relay 2 input to 50%

### 3. Heater Setpoint (`H`)

- **Format**: `H <heater> <setpoint>`
  - **Example**: `H 1 150.5` to set heater 1's temperature setpoint to 150.5°C
  - **Example**: `H 2 200` to set heater 2's temperature setpoint to 200°C

### 4. PID Tuning (`P`)

- **Format**: `P <heater> <Kp>,<Ki>,<Kd>`
  - **Example**: `P 1 2.5,0.1,0.01` to update PID values for heater 1 (Kp=2.5, Ki=0.1, Kd=0.01)
  - **Example**: `P 2 3.0,0.2,0.02` to update PID values for heater 2 (Kp=3.0, Ki=0.2, Kd=0.02)

### 5. Thermo Select (`S`)

- **Format**: `S <relay> <thermo>`
  - **Example**: `S 1 1` to set relay 1 to read from thermocouple 1
  - **Example**: `S 2 2` to set relay 2 to read from thermocouple 2

### Invalid Commands

- Any command that does not match the above patterns will result in an error message: `Invalid command!`

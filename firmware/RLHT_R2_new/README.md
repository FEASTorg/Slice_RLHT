# RLHT Firmware Docs

Notes on the firmware for relay heater slice.

## Command Structure

| Command | Target | Data Format           | Description                                                                                                                                                                 |
| ------- | ------ | --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `T`     | `1/2`  | -                     | Request temperature reading from thermocouple 1 or 2.                                                                                                                       |
| `H`     | `1/2`  | `setpoint,thermo,dir` | Set heater `1` or `2` with a target temperature (`setpoint`), select thermocouple (`thermo` = `1` or `2`), and control direction (`dir` = `0` for direct, `1` for reverse). |
| `P`     | `1/2`  | `Kp,Ki,Kd`            | Set PID tuning parameters (`Kp`, `Ki`, `Kd`) for heater `1` or `2`.                                                                                                         |

### Data Format Details

- **`setpoint`**: Target temperature in °C (float).
- **`thermo`**: Selects thermocouple for PID input (`1` = CH1, `2` = CH2).
- **`dir`**: Controller direction (`0` = Direct, `1` = Reverse).
- **`Kp, Ki, Kd`**: PID tuning parameters (float values).

### Example Commands

| Command            | Meaning                                                                  |
| ------------------ | ------------------------------------------------------------------------ |
| `H,1,100.0,2,0`    | Set heater 1 to **100°C**, using **thermocouple 2**, in **direct mode**. |
| `P,2,2.5,0.1,0.05` | Set PID for heater 2 with **Kp = 2.5, Ki = 0.1, Kd = 0.05**.             |

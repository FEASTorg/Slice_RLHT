#ifndef WIRE_COMMS_H
#define WIRE_COMMS_H

struct RLHT_t
{
    double heatSetpoint_1 = 0; // set desired temperature for heater 1 (if using relay as heater actuator)
    double relay1Input;        // input to relay 1 taken from selected thermocouple
    double rOnTime_1;          // time relay is on in ms (0-period)
    int rPeriod_1 = 2000;      // duration of relay 1 cycle in ms (1000ms = 1s)
    double Kp_1 = 0;           // PID proportional gain for heater 1
    double Ki_1 = 0;           // PID integral gain for heater 1
    double Kd_1 = 0;           // PID derivative gain for heater 1
    double heatSetpoint_2 = 0;
    double relay2Input;
    double rOnTime_2;
    int rPeriod_2 = 2000;
    double Kp_2 = 0;
    double Ki_2 = 0;
    double Kd_2 = 0;
    double thermo1; // thermocouple 1 measurement
    double thermo2;
    char thermoSelect[2] = {1, 2}; // select which thermocouples pair with relays {relay1, relay2}
};

#endif // WIRE_COMMS_H
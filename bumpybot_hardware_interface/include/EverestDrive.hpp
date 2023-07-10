//
// Created by Carlos on 7/14/22.
//

#ifndef SRC_EVERESTDRIVE_HPP
#define SRC_EVERESTDRIVE_HPP

struct EverestDriveRPDOA{
    uint16_t control_word;            // 0x6040
    uint8_t operation_mode;           // 0x6060
    int32_t desired_motor_current;    // 0x201A (quadrature set point)
};

struct EverestDriveTPDOA{
    uint16_t control_word;            // 0x6041
    int32_t motor_position;           // 0x6064 (32-bit encoder tick counts)
    int32_t motor_velocity;           // 0x606C
    uint8_t operation_mode_display;   // 0x6061
    int16_t joint_torque;             // 0x6077
    int32_t current_demand;           // 0x2072 (current quadrature demand)
//    uint32_t state;         // uint32           0x6061 4bit FSM stste, 4bit MOP, 24bit,warnnings, errors, fatals
//    uint16_t temperature;   // float C          0x2002
};

#endif //SRC_EVERESTDRIVE_HPP

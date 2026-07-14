#ifndef BIPED_MAPPING_HPP_
#define BIPED_MAPPING_HPP_

namespace biped_mapping {

    // ---------------------------------------------------------
    // COMMAND PACKET INDICES (ROS 2 -> STM32)
    // Array Size: 13 Elements
    // ---------------------------------------------------------
    namespace cmd {
        constexpr int LIN_REF_0   = 0;
        constexpr int LIN_REF_1   = 1;
        constexpr int LIN_REF_2   = 2;
        constexpr int LIN_REF_3   = 3;
        constexpr int LIN_REF_4   = 4;
        constexpr int LIN_REF_5   = 5;
        
        constexpr int RMD_REF_0   = 6;
        constexpr int RMD_REF_1   = 7;
        
        constexpr int SERVO_REF_0 = 8;
        constexpr int SERVO_REF_1 = 9;
        constexpr int SERVO_REF_2 = 10;
        
        constexpr int DC_MOTOR_1  = 11;
        constexpr int DC_MOTOR_2  = 12;
    }

    // ---------------------------------------------------------
    // FEEDBACK PACKET INDICES (STM32 -> ROS 2)
    // Array Size: 12 Elements
    // ---------------------------------------------------------
    namespace fb {
        constexpr int LIN_ACTUAL_0   = 0;
        constexpr int LIN_ACTUAL_1   = 1;
        constexpr int LIN_ACTUAL_2   = 2;
        constexpr int LIN_ACTUAL_3   = 3;
        constexpr int LIN_ACTUAL_4   = 4;
        constexpr int LIN_ACTUAL_5   = 5;
        
        constexpr int RMD_ACTUAL_0   = 6;
        constexpr int RMD_ACTUAL_1   = 7;
        
        constexpr int IMU_ROLL       = 8;
        constexpr int IMU_PITCH      = 9;
        constexpr int IMU_ROLL_RATE  = 10;
        constexpr int IMU_PITCH_RATE = 11;
    }

} // namespace biped_mapping

#endif // BIPED_MAPPING_HPP_

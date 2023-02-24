// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace t34 
{
    constexpr int kDriverControllerPort = 0;

    //  Swerve Mappings     //
    #define ID_LEFT_FWD_DRIVE       22
    #define ID_LEFT_FWD_STEER       23
    #define ID_RIGHT_FWD_DRIVE      32
    #define ID_RIGHT_FWD_STEER      33
    #define ID_LEFT_AFT_DRIVE       42
    #define ID_LEFT_AFT_STEER       43
    #define ID_RIGHT_AFT_DRIVE      52
    #define ID_RIGHT_AFT_STEER      53

    //  Joystick Mappings   //
    #define ID_DRIVE_CONTROLLER     0
    #define ID_MECH_CONTROLLER      1

    //  Arm Motor Mappings  //
    #define ID_WRIST_ROT_MOTOR      11 //talon 7
    #define ID_ARM_MOTOR            12 
    #define ID_WRIST_Y_MOTOR        13 //talon 2
    #define ID_ARM_EXT_MOTOR        14

    //  Ramp Limiters       //
    #define RAMP_LIMIT_X            1.0
    #define RAMP_LIMIT_Y            1.0
    #define RAMP_LIMIT_R            1.0

    //  Encoder IDs         //
    #define ID_ENCODER_LEFT_FWD        21
    #define ID_ENCODER_RIGHT_FWD       31
    #define ID_ENCODER_LEFT_AFT        41
    #define ID_ENCODER_RIGHT_AFT       51

    //  Commands            //



    constexpr double PI{ 3.1415926535897932384626433832 };

    constexpr double _180_DIV_PI(){ return 180.0 / PI; }

    constexpr double _PI_DIV_180(){ return PI / 180.0; }

    //constexpr double FULL_UNITS{ 26214.4 };
    constexpr double FULL_UNITS{ 4096.0 };
    //constexpr double UNITS_PER_INCH{ 1331.52715655 };
    constexpr double UNITS_PER_INCH{ 208.051118211 };
    constexpr double ABS_TO_IS{  FULL_UNITS/4096.0  };
    constexpr double inches_to_mm(double inches) { return inches / 0.03937008; }
    /*
        8.14
        6.75
        6.12
        5.12
    */

    constexpr double FRAME_LENGTH{ 22.5 };
    constexpr double FRAME_WIDTH{ 22.5 };
//    constexpr double FRAME_LENGTH{ 736.6 };
//    constexpr double FRAME_WIDTH{ 736.6 };

    constexpr double _45_DEG( 4608.0);
    constexpr double _135_DEG( 3 * 4608.0); 
    constexpr double _225_DEG( 5 * 4608.0);
    constexpr double _315_DEG( 7 * 4608.0);

    constexpr double RATIO() { 
                return sqrt(
                    (FRAME_LENGTH * FRAME_LENGTH) + 
                    (FRAME_WIDTH * FRAME_WIDTH)); 
            }
            
    constexpr double FRAME_LENGTH_DIV_RATIO() 
        { 
            return FRAME_LENGTH / RATIO(); 
        }
            
    constexpr double FRAME_WIDTH_DIV_RATIO() 
    { 
                return FRAME_WIDTH / RATIO(); 
            }

    inline double sqr(double d) { return d * d; }
    inline double deg_to_rad(double deg) { return deg * _PI_DIV_180(); }
    inline double rad_to_deg(double rad) { return rad * _180_DIV_PI(); }

}  

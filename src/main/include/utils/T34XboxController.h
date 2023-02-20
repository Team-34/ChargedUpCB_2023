#pragma once

#include <frc/XboxController.h>

namespace t34 {

    // Enumeration for specifying the target axis in a generic function
    enum class AxisType {
        Trigger,
        XAxis,
        YAxis,
        ZAxis
    };

    enum class JoystickHand {
    	Left,
    	Right,
    };

    // Subclass of the frc::XboxController adding extra functionality
    class T34XboxController : public frc::XboxController {
    public: // PUBLIC METHODS
        T34XboxController(int32_t port);

    	void setAllAxisDeadband(double value);
        void setAxisDeadband(JoystickHand hand, AxisType axis, double value);

        double getXDB(JoystickHand hand) const;
        double getYDB(JoystickHand hand) const;
        double getTriggerDB(JoystickHand hand) const;
    	double getTriggersCoercedDB() const;
    	
    	inline double getLeftStickXDB() const   { return getXDB(JoystickHand::Left); }
    	inline double getLeftStickYDB() const   { return getYDB(JoystickHand::Left); }
    	inline double getRightStickXDB() const  { return getXDB(JoystickHand::Right); }
    	inline double getRightStickYDB() const  { return getYDB(JoystickHand::Right); }
    	inline double getLeftTriggerDB() const  { return getTriggerDB(JoystickHand::Left); }
    	inline double getRightTriggerDB() const { return getTriggerDB(JoystickHand::Right); }
    	
    private: //PRIVATE DATA
        double m_left_x_db;
        double m_left_y_db;
        double m_right_x_db;
        double m_right_y_db;
        double m_left_trigger_db;
        double m_right_trigger_db;
    };

}

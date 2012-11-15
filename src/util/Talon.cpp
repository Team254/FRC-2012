#include "Talon.h"

/**
 * Common initialization code called by all constructors.
 * 
 * Note that the Talon uses the following bounds for PWM values.  These values were determined
 * empirically through experimentation during the 2008 beta testing of the new control system.
 * Testing during the beta period revealed a significant amount of variation between Talons.
 * The values below are chosen to ensure that teams using the default values should be able to
 * get "full power" with the maximum and minimum values.  For better performance, teams may wish
 * to measure these values on their own Talons and set the bounds to the particular values
 * measured for the actual Talons they were be using. 
 *   - 210 = full "forward"
 *   - 138 = the "high end" of the deadband range
 *   - 132 = center of the deadband range (off)
 *   - 126 = the "low end" of the deadband range
 *   - 56 = full "reverse"
 */
void Talon::InitTalon()
{
	// TODO: find these values
	SetBounds(210, 138, 132, 126, 56);
	SetPeriodMultiplier(kPeriodMultiplier_2X);
	SetRaw(m_centerPwm);

	//nUsageReporting::report(nUsageReporting::kResourceType_Talon, GetChannel(), GetModuleNumber() - 1);
}

Talon::Talon(UINT32 channel) : SafePWM(channel)
{
	InitTalon();
}

Talon::Talon(UINT8 moduleNumber, UINT32 channel) : SafePWM(moduleNumber, channel)
{
	InitTalon();
}

Talon::~Talon()
{
}

void Talon::Set(float speed, UINT8 syncGroup)
{
	double goal_speed = speed;
	double a1_ = 0.394864;
	double a3_ = 0.67367;
	double a5_ = -3.409659;
	double a7_ = 4.197125;
	double deadband_ = 0.082;

	if(goal_speed > deadband_) {
        goal_speed -= deadband_;
    }
    else if(goal_speed < -deadband_) {
        goal_speed += deadband_;
    }
    else {
        goal_speed = 0.0;
    }
    goal_speed = goal_speed / (1.0 - deadband_);

    double goal_speed2 = goal_speed * goal_speed;
    double goal_speed3 = goal_speed2 * goal_speed;
    double goal_speed4 = goal_speed3 * goal_speed;
    double goal_speed5 = goal_speed4 * goal_speed;
    double goal_speed6 = goal_speed5 * goal_speed;
    double goal_speed7 = goal_speed6 * goal_speed;

    double answer_7th_order = (a7_ * goal_speed7 + a5_ * goal_speed5 +
                               a3_ * goal_speed3 + a1_ * goal_speed);

    // Average polynomial with line
    double answer = 0.85 * 0.5 * (answer_7th_order)
            + .15 * goal_speed * (1.0 - deadband_);

    if(answer > 0.001) {
        answer += deadband_;
    }
    else if(answer < -0.001) {
        answer -= deadband_;
    }

	SetSpeed(answer);
}

float Talon::Get()
{
	return GetSpeed();
}

void Talon::Disable()
{
	SetRaw(kPwmDisabled);
}

void Talon::PIDWrite(float output) 
{
	Set(output);
}
#ifndef Talon_H
#define Talon_H

#include "WPILib.h"

/**
 * Cross the Road Electronics
 * Talon Speed Controller
 */
class Talon : public SafePWM, public SpeedController, public PIDOutput
{
public:
	explicit Talon(UINT32 channel);
	Talon(UINT8 moduleNumber, UINT32 channel);
	virtual ~Talon();
	virtual void Set(float value, UINT8 syncGroup=0);
	virtual float Get();
	virtual void Disable();

	virtual void PIDWrite(float output);

private:
	void InitTalon();
};

#endif

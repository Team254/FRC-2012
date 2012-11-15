#include "Talon.h"

/**
 *  These values are not up to date yet!
 *   - 205 = full "forward"
 *   - 155 = the "high end" of the deadband range
 *   - 150 = center of the deadband range (off)
 *   - 145 = the "low end" of the deadband range
 *   - 85 = full "reverse"
 */
void Talon::InitTalon() {
	// TODO: update these values!
	SetBounds(205, 155, 150, 145, 85); // for now
	SetPeriodMultiplier(kPeriodMultiplier_2X);
	SetRaw(m_centerPwm);

	nUsageReporting::report(nUsageReporting::kResourceType_Victor, GetChannel(), GetModuleNumber() - 1);
}

Talon::Talon(UINT32 channel) : SafePWM(channel) {
	InitTalon();
}

Talon::Talon(UINT8 moduleNumber, UINT32 channel) : SafePWM(moduleNumber, channel) {
	InitTalon();
}

Talon::~Talon() {
}

void Talon::Set(float speed, UINT8 syncGroup) {
	SetSpeed(speed);
}

float Talon::Get() {
	return GetSpeed();
}

void Talon::Disable() {
	SetRaw(kPwmDisabled);
}

void Talon::PIDWrite(float output) {
	Set(output);
}

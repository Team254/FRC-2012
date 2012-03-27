#ifndef CONTINUOUS_ACCEL_FILTER_H
#define CONTINUOUS_ACCEL_FILTER_H

#include "AccelFilterBase.hpp"

/**
 * A continuous time acceleration profile.
 *
 * An instance of continuous data is assumed to have a constant
 * rate of change over a period of time. Thus, over any given time increment,
 * acceleration is assumed to be constant. Thus data for this filter does not
 * need to be polled necessarily periodically.
 */
class ContinuousAccelFilter : public AccelFilterBase {
	
public:
	
    ContinuousAccelFilter(double currPos=0, double currVel=0, double currAcc=0);
	virtual ~ContinuousAccelFilter() {}
    virtual void CalcSystem(double distance_to_target, double v, double goal_v, double max_a, double max_v, double dt);

protected:
    virtual void UpdateVals(double acc, double dt);   
	
private:
    void maxAccelTime(double distance_left, double curr_vel, double goal_vel, double max_a, double max_v, double& t1, double& a1, double& ct, double& t2, double& a2);
};
#endif

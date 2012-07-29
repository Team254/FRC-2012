#include "ContinuousAccelFilter.h"
#include <math.h>
#include <assert.h>
ContinuousAccelFilter::ContinuousAccelFilter(double currPos, double currVel, double currAcc)
: AccelFilterBase(currPos, currVel, currAcc)
{}

void ContinuousAccelFilter::UpdateVals(double acc, double dt) {
    // basic kinematics
    m_currPos += m_currVel * dt + acc * .5 * dt * dt;
    m_currVel += acc * dt;
    m_currAcc = acc;
}

void ContinuousAccelFilter::CalcSystem(double distance_to_target, double v, double goal_v, double max_a, double max_v, double dt)
{
  double dt2, a, const_time, dtf, af;
  maxAccelTime(distance_to_target, v, goal_v, max_a, max_v, dt2, a, const_time, dtf, af);
  double time_left = dt;
  if (dt2 > time_left)
    UpdateVals(a, time_left);
  else {
    UpdateVals(a, dt2);
    time_left -= dt2;
    if (const_time > time_left)
      UpdateVals(0, time_left);
    else {
      UpdateVals(0, const_time);
      time_left -= const_time;
      if (dtf > time_left)
        UpdateVals(af, time_left);
      else {
        UpdateVals(af, dtf);
        time_left -= dtf;
        UpdateVals(0, time_left);
      }
    }
  }
}
void ContinuousAccelFilter::maxAccelTime(double distance_left, double curr_vel, double goal_vel, double max_a, double max_v, double& t1, double& a1, double& ct, double& t2, double& a2)
{
  double const_time = 0;
  double start_a = 0;
  if (distance_left > 0) {
    start_a = max_a;
  } else if (distance_left == 0) {
    t1 = 0;
    a1 = 0;
    ct = 0;
    t2 = 0;
    a2 = 0;
    return;
  } else {
    maxAccelTime(-distance_left, -curr_vel, -goal_vel, max_a, max_v, t1, a1, ct, t2, a2);
    a1 *= -1;
    a2 *= -1;
    return;
  }
  double max_accel_velocity = distance_left * 2 * fabs(start_a) + curr_vel * curr_vel;
  if (max_accel_velocity > 0)
    max_accel_velocity = sqrt(max_accel_velocity);
  else
    max_accel_velocity = -sqrt(-max_accel_velocity);

  // Since we know what we'd have to do if we kept after it to decelerate, we know the sign of the acceleration.
  double final_a;
  if (max_accel_velocity>goal_vel)
    final_a = -max_a;
  else
    final_a = max_a;

  // We now know the top velocity we can get to
  double top_v = sqrt((distance_left + (curr_vel * curr_vel) / (2.0 * start_a) + (goal_vel * goal_vel) / (2.0 * final_a)) / (-1.0 / (2.0 * final_a) + 1.0 / (2.0 * start_a)));

  // If it is too fast, we now know how long we get to accelerate for and how long to go at constant velocity
  double accel_time=0;
  if (top_v > max_v) {
    accel_time = (max_v - curr_vel) / max_a;
    const_time = (distance_left + (goal_vel * goal_vel - max_v * max_v) / (2.0 * max_a)) / max_v;
  }
  else
    accel_time = (top_v - curr_vel) / start_a;

  assert(top_v > -max_v);

  t1 = accel_time;
  a1 = start_a;
  ct = const_time;
  t2 = (goal_vel-top_v) / final_a;
  a2 = final_a;
}


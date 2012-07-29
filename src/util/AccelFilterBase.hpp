#ifndef ACCEL_FILTER_BASE_H
#define ACCEL_FILTER_BASE_H

/**
 * A basic interface for different acceleration profiles.
 *
 * Calculates the kinematic physics of a system by determining distance
 * remaining and maximum, intended, and current velocity and acceleration.
 */
class AccelFilterBase {

public:

  AccelFilterBase(double currPos = 0, double currVel = 0, double currAcc = 0)
  : m_currPos(currPos)
  , m_currVel(currVel)
  , m_currAcc(currAcc)
  {}
  virtual ~AccelFilterBase() {}

  // Getter functions
  virtual const double GetCurrPos() const {return m_currPos;}
  virtual const double GetCurrVel() const {return m_currVel;}
  virtual const double GetCurrAcc() const {return m_currAcc;}

  // Recalculate the system
  virtual void CalcSystem(double distance_to_target, double v, double goal_v, double max_a, double max_v, double dt) = 0;

protected:
  // default value updater
  virtual void UpdateVals(double acc, double dt) {
    m_currAcc = acc;
    m_currVel += m_currAcc;
    m_currPos += m_currVel * dt;
    m_currAcc /= dt;
  }

  // current vars
  double m_currPos;
  double m_currVel;
  double m_currAcc;
};
#endif

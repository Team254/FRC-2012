/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "DriverStation.h"
#include "CheesyRobot.h"
#include <taskLib.h>

const double CheesyRobot::kDefaultPeriod;

/**
 * Constructor for RobotCheesyBase
 * 
 * The constructor initializes the instance variables for the robot to indicate
 * the status of initialization for disabled, autonomous, and teleop code.
 */
CheesyRobot::CheesyRobot()
	: m_disabledInitialized (false)
	, m_autonomousInitialized (false)
	, m_teleopInitialized (false)
	, m_period (kDefaultPeriod)
{
	m_watchdog.SetEnabled(false);
}

/**
 * Free the resources for a RobotCheesyBase class.
 */
CheesyRobot::~CheesyRobot()
{
}

/**
 * Set the period for the periodic functions.
 * 
 * @param period The period of the periodic function calls.  0.0 means sync to driver station control data.
 */
void CheesyRobot::SetPeriod(double period)
{

    // Not syncing with the DS, so start the timer for the main loop
	m_mainLoopTimer.Reset();
    m_mainLoopTimer.Start();
	
	m_period = period;
}

/**
 * Get the period for the periodic functions.
 * Returns 0.0 if configured to syncronize with DS control data packets.
 * @return Period of the periodic function calls
 */
double CheesyRobot::GetPeriod()
{
	return m_period;
}

/**
 * Get the number of loops per second for the CheesyRobot
 * @return Frequency of the periodic function calls
 */
double CheesyRobot::GetLoopsPerSec()
{
	return 1.0 / m_period;
}

/**
 * Provide an alternate "main loop" via StartCompetition().
 * 
 * This specific StartCompetition() implements "main loop" behavior like that of the FRC
 * control system in 2008 and earlier, with a primary (slow) loop that is
 * called periodically, and a "fast loop" (a.k.a. "spin loop") that is 
 * called as fast as possible with no delay between calls. 
 */
void CheesyRobot::StartCompetition()
{
	// first and one-time initialization
	RobotInit();
	static double t = 0;
	// loop forever, calling the appropriate mode-dependent function
	while (TRUE)
	{
		// Call the appropriate function depending upon the current robot mode
		if (IsDisabled())
		{
			// call DisabledInit() if we are now just entering disabled mode from
			// either a different mode or from power-on
			if(!m_disabledInitialized)
			{
				DisabledInit();
				m_disabledInitialized = true;
				// reset the initialization flags for the other modes
				m_autonomousInitialized = false;
				m_teleopInitialized = false;
			}
			if (NextPeriodReady())
			{
				FRC_NetworkCommunication_observeUserProgramDisabled();
				DisabledPeriodic();
			}
			DisabledContinuous();
		}
		else if (IsAutonomous())
		{
			// call AutonomousInit() if we are now just entering autonomous mode from
			// either a different mode or from power-on
			if(!m_autonomousInitialized)
			{
				AutonomousInit();
				m_autonomousInitialized = true;
				// reset the initialization flags for the other modes
				m_disabledInitialized = false;
				m_teleopInitialized = false;
			}
			if (NextPeriodReady())
			{
				FRC_NetworkCommunication_observeUserProgramDisabled();
				AutonomousPeriodic();
			}
			AutonomousContinuous();
		}
		else
		{
			// call TeleopInit() if we are now just entering teleop mode from
			// either a different mode or from power-on
			if(!m_teleopInitialized)
			{
				TeleopInit();
				m_teleopInitialized = true;
				// reset the initialization flags for the other modes
				m_disabledInitialized = false;
				m_autonomousInitialized = false;
			}
			if (NextPeriodReady())
			{
				FRC_NetworkCommunication_observeUserProgramDisabled();
				TeleopPeriodic();
			}
			TeleopContinuous();
		}
	}	
}

/**
 * Determine if the periodic functions should be called.
 *
 * If m_period > 0.0, call the periodic function every m_period as compared
 * to Timer.Get().  If m_period == 0.0, call the periodic functions whenever
 * a packet is received from the Driver Station, or about every 20ms.
 *
 * @todo Decide what this should do if it slips more than one cycle.
 */

bool CheesyRobot::NextPeriodReady()
{
    return m_mainLoopTimer.HasPeriodPassed(m_period);
}

/**
 * Robot-wide initialization code should go here.
 * 
 * Users should override this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void CheesyRobot::RobotInit()
{
	printf("Default %s() method... Overload me!\n", __FUNCTION__);
}

/**
 * Initialization code for disabled mode should go here.
 * 
 * Users should override this method for initialization code which will be called each time
 * the robot enters disabled mode.
 */
void CheesyRobot::DisabledInit()
{
	printf("Default %s() method... Overload me!\n", __FUNCTION__);
}

/**
 * Initialization code for autonomous mode should go here.
 * 
 * Users should override this method for initialization code which will be called each time
 * the robot enters autonomous mode.
 */
void CheesyRobot::AutonomousInit()
{
	printf("Default %s() method... Overload me!\n", __FUNCTION__);
}

/**
 * Initialization code for teleop mode should go here.
 * 
 * Users should override this method for initialization code which will be called each time
 * the robot enters teleop mode.
 */
void CheesyRobot::TeleopInit()
{
	printf("Default %s() method... Overload me!\n", __FUNCTION__);
}

/**
 * Periodic code for disabled mode should go here.
 * 
 * Users should override this method for code which will be called periodically at a regular
 * rate while the robot is in disabled mode.
 */
void CheesyRobot::DisabledPeriodic()
{
	static bool firstRun = true;
	if (firstRun)
	{
		printf("Default %s() method... Overload me!\n", __FUNCTION__);
		firstRun = false;
	}
	taskDelay(1);
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Users should override this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void CheesyRobot::AutonomousPeriodic()
{
	static bool firstRun = true;
	if (firstRun)
	{
		printf("Default %s() method... Overload me!\n", __FUNCTION__);
		firstRun = false;
	}
	taskDelay(1);
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Users should override this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void CheesyRobot::TeleopPeriodic()
{
	static bool firstRun = true;
	if (firstRun)
	{
		printf("Default %s() method... Overload me!\n", __FUNCTION__);
		firstRun = false;
	}
	taskDelay(1);
}

/**
 * Continuous code for disabled mode should go here.
 *
 * Users should override this method for code which will be called repeatedly as frequently
 * as possible while the robot is in disabled mode.
 */
void CheesyRobot::DisabledContinuous()
{
	static bool firstRun = true;
	if (firstRun)
	{
		printf("Default %s() method... Overload me!\n", __FUNCTION__);
		firstRun = false;
	}
	m_ds->WaitForData();
}

/**
 * Continuous code for autonomous mode should go here.
 *
 * Users should override this method for code which will be called repeatedly as frequently
 * as possible while the robot is in autonomous mode.
 */
void CheesyRobot::AutonomousContinuous()
{
	static bool firstRun = true;
	if (firstRun)
	{
		printf("Default %s() method... Overload me!\n", __FUNCTION__);
		firstRun = false;
	}
	m_ds->WaitForData();
}

/**
 * Continuous code for teleop mode should go here.
 *
 * Users should override this method for code which will be called repeatedly as frequently
 * as possible while the robot is in teleop mode.
 */
void CheesyRobot::TeleopContinuous()
{
	static bool firstRun = true;
	if (firstRun)
	{
		printf("Default %s() method... Overload me!\n", __FUNCTION__);
		firstRun = false;
	}
	m_ds->WaitForData();
}

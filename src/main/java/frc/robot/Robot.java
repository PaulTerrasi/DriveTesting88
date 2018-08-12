/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation.
 */
public class Robot extends IterativeRobot {

  // Decides which operator interface to use
  IOperatorInterface mOI;

  /**
   * Runs when the robot is first powered on.
   */
  @Override
  public void robotInit() {
    mOI = new OperatorInterfaceImpl();
  }

  /**
   * Runs whenever a driver station packet is received, directly following
   * the mode specific periodic functions (disabledPeriodic(), 
   * teleopPeriodic(), etc.).
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * Runs whenever the robot receives a packet during teleop mode.
   */
  @Override
  public void teleopPeriodic() {
  }
}

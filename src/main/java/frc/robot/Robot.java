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

   // This is used to select what drive mode will be used
  private enum DriveMode {
    BASIC_OPEN_LOOP,
    BEST_OPEN_LOOP,
    CLOSED_LOOP,
    CALIBRATE_MIN_THROTTLE,
    CALIBRATE_MIN_TURN_STILL,
    CALIBRATE_MIN_TURN_MOVING
  }
  private final DriveMode driveMode = DriveMode.BEST_OPEN_LOOP;

  // Decides which operator interface to use
  IOperatorInterface mOI;

  // Drivetrain controller
  Drive mDrive;

  /**
   * Runs when the robot is first powered on.
   */
  @Override
  public void robotInit() {
    mOI = new LogitechControllerOI();
    mDrive = Drive.getInstance();
  }

  
  int loopCount = 0; // The number of loops that have passes
  /**
   * Runs whenever a driver station packet is received, directly following
   * the mode specific periodic functions (disabledPeriodic(), 
   * teleopPeriodic(), etc.).
   */
  @Override
  public void robotPeriodic() {
    double driveThrottle = mOI.getThrottle();
    double driveTurn = mOI.getTurn();
    
    switch (driveMode) {
      case BASIC_OPEN_LOOP:
        mDrive.basicArcade(driveThrottle, driveTurn);
        break;
      case BEST_OPEN_LOOP:
        mDrive.bestOpenLoopDrive(driveThrottle, driveTurn);
        break;
      case CLOSED_LOOP:
        break;
      case CALIBRATE_MIN_THROTTLE:
        // Slowly increment the thottle of the drivetrain from 0
        double curThrottle = (loopCount / 100) * .001;
        
        if (loopCount % 100 == 0) {
          System.out.println("Current speed: " + curThrottle + "\n");
        }
        
        mDrive.basicArcade(curThrottle, 0);
        break;
      case CALIBRATE_MIN_TURN_STILL:
        // Slowly increment the turn of the drivetrain from 0
    	  double curTurnStill = (loopCount / 100) * .001;
	  
        if (loopCount % 100 == 0) {
          System.out.println("Current speed: " + curTurnStill + "\n");
        }
        
        mDrive.basicArcade(0, curTurnStill);
        break;
      case CALIBRATE_MIN_TURN_MOVING:
        // Slowly increment the turn of the drivetrain from 0 while moving,
        // occasionally changing direction
    	  double curTurnMoving = (loopCount / 100) * .001;

        if (loopCount % 100 == 0) {
          System.out.println("Current speed: " + curTurnMoving + "\n");
        }
        
        double throttle = .2;
        if (loopCount % 400 >= 200) {
          throttle = -.2;
        }
        
        mDrive.basicArcade(throttle, mDrive.cheesifyTurn(throttle, curTurnMoving));
    }
    loopCount++;
	  
//	  double power = (powerCount / 100) * .001;
//	  
//	  if (powerCount % 100 == 0) {
//		  System.out.println("Current speed: " + power + "\n");
//	  }
//	  
//	  mDrive.basicArcade(power, 0);
//	  powerCount++;
    
//	  double power = (powerCount / 100) * .001;
//	  
//	  if (powerCount % 100 == 0) {
//		  System.out.println("Current speed: " + power + "\n");
//	  }
//	  
//	  mDrive.basicArcade(0, power);
//	  powerCount++;
	  
//	  double power = (powerCount / 100) * .001;
//	  
//	  if (powerCount % 100 == 0) {
//		  System.out.println("Current speed: " + power + "\n");
//	  }
//	  
//	  double throttle = .2;
//	  if (powerCount % 400 >= 200) {
//		  throttle = -.2;
//	  }
//	  
//	  mDrive.basicArcade(throttle, mDrive.cheesifyTurn(throttle, power));
//	  powerCount++;
  }

  /**
   * Runs whenever the robot receives a packet during teleop mode.
   */
  @Override
  
  public void teleopPeriodic() {
  }
}
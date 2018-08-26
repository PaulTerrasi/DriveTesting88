package frc.robot;

/**
 * Defines how the inputs given by a robot driver or operator correspond to
 * different signals in the robot code.
 */
interface IOperatorInterface {

    /**
     * Returns the value that the driver is commanding for drivetrain throttle 
     */
    public double getThrottle();
    
    /**
     * Returns the value that the driver is commanding for drivetrain turn rate
     */
    public double getTurn();

    // Sets the given value to zero if its magnitude is less than the DEADBAND,
    // otherwise linearly scale the value so that if value == deadband it is
    // brought down to 0, but if value == max it stays the same
	public static double applyDeadband(
            double rawAxis, double deadband, double max) {

		if (Math.abs(rawAxis) < deadband) {
            return 0;
        } else {
            return (rawAxis - deadband) * max / (max - deadband);
        }
	}
}
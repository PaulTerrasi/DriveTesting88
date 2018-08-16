package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Standard implementation of the operator interface.
 */
class LogitechControllerOI implements IOperatorInterface {

	private static final int DRIVER_JOYSTICK_PORT = 2;
	private static final double DEADBAND = 0.025;

    private Joystick mDriverJoystick;

    public LogitechControllerOI() {
        mDriverJoystick = new Joystick(DRIVER_JOYSTICK_PORT);
    }

	@Override
	public double getThrottle() {
		return IOperatorInterface.applyDeadband(mDriverJoystick.getRawAxis(1), 
				DEADBAND, 1);
	}

	@Override
	public double getTurn() {
		return IOperatorInterface.applyDeadband(mDriverJoystick.getRawAxis(5), 
				DEADBAND, 1);
	}

}
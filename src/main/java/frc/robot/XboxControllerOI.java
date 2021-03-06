package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Standard implementation of the operator interface.
 */
class XboxControllerOI implements IOperatorInterface {

	private static final int DRIVER_JOYSTICK_PORT = 0;
	private static final double DEADBAND = 0.15;

    private Joystick mDriverJoystick;

    public XboxControllerOI() {
        mDriverJoystick = new Joystick(DRIVER_JOYSTICK_PORT);
    }

	@Override
	public double getThrottle() {
		return IOperatorInterface.applyDeadband(mDriverJoystick.getRawAxis(5), 
				DEADBAND, 1);
	}

	@Override
	public double getTurn() {
		return IOperatorInterface.applyDeadband(mDriverJoystick.getRawAxis(0), 
				DEADBAND, 1);
	}

}
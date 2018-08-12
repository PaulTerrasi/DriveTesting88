package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Standard implementation of the operator interface.
 */
class OperatorInterfaceImpl implements IOperatorInterface {

    private static final int DRIVER_JOYSTICK_PORT = 0;

    Joystick mDriverJoystick;

    public OperatorInterfaceImpl() {
        mDriverJoystick = new Joystick(DRIVER_JOYSTICK_PORT);
    }

	@Override
	public double getThrottle() {
		return mDriverJoystick.getRawAxis(0);
	}

	@Override
	public double getTurn() {
		return mDriverJoystick.getRawAxis(1);
	}

}
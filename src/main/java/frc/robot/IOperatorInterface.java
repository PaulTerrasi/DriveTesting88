package frc.robot;

/**
 * Defines how the inputs given by a robot driver or operator correspond to
 * different signals in the robot code.
 */
interface IOperatorInterface {

    public double getThrottle();
    
    public double getTurn();
}
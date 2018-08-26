package frc.robot;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Handles the control of the drivetrain.
 */
class Drive {

    // The singleton instance of the Drive
    private static Drive instance;

    // The number of motors per side
    private static final int MOTORS_PER_SIDE = 4;

    // CAN IDs of the drive motor controllers
    private static final int LEFT_MASTER_CAN_ID = 31;
    private static final int RIGHT_MASTER_CAN_ID = 32;
    private static final int[] LEFT_FOLLOWER_CAN_IDS = {26, 24, 25};
    private static final int[] RIGHT_FOLLOWER_CAN_IDS = {22, 23, 21};

    // The drive motor controller objects
    private TalonSRX leftMasterTalon;
    private TalonSRX rightMasterTalon;
    private VictorSPX[] leftFollowerVictors = new VictorSPX[MOTORS_PER_SIDE - 1];
    private VictorSPX[] rightFollowerVictors = new VictorSPX[MOTORS_PER_SIDE - 1];

    // Drive talon config values
    private static final FeedbackDevice TALON_FEEDBACK_SENSOR = FeedbackDevice.QuadEncoder;
    private static final int TALON_PID_IDX = 0;
    private static final int TALON_KP = 0;
    private static final int TALON_KI = 0;
    private static final int TALON_KD = 0;
    private static final int TALON_KF = 1;

    private static final int TALON_CONFIG_TIMEOUT = 1; // ms
    private static final double TALON_OPEN_LOOP_RAMP = 1/8; // seconds from 0 to full
    private static final double TALON_CLOSED_LOOP_RAMP = 1/8; // seconds from 0 to full
    private static final double TALON_MAX_FORWARD_VOLTAGE = 1; // percent vbus
    private static final double TALON_MAX_REVERSE_VOLTAGE = -1; // percent vbus
    private static final double TALON_MIN_FORWARD_VOLTAGE = 0; // percent vbus
    private static final double TALON_MIN_REVERSE_VOLTAGE = 0; // percent vbus
    private static final double TALON_NEUTRAL_DEADBAND = 0.01; // percent vbus
    private static final double TALON_VOLTAGE_COMP_SATURATION = 0; // volts
    private static final int TALON_VOLTAGE_FILTER_WINDOW = 32; // samples
    private static final double TALON_FEEDBACK_COEFFICIENT = 1; // ??
    private static final VelocityMeasPeriod TALON_VELOCITY_PERIOD = VelocityMeasPeriod.Period_100Ms;
    private static final LimitSwitchSource TALON_LIMIT_SWITCH_SOURCE = LimitSwitchSource.Deactivated;
    private static final LimitSwitchNormal TALON_LIMIT_SWITCH_NORMAL = LimitSwitchNormal.NormallyOpen;
    private static final int TALON_FORWARD_SOFT_LIMIT = 0; // Encoder ticks
    private static final int TALON_REVERSE_SOFT_LIMIT = 0; // Encoder ticks
    private static final boolean TALON_FORWARD_SOFT_LIMIT_ENABLE = false;
    private static final boolean TALON_REVERSE_SOFT_LIMIT_ENABLE = false;
    private static final int TALON_INTEGRAL_ZONE = 0;
    private static final int TALON_MIN_CLOSED_LOOP_ERROR = 0;
    private static final double TALON_MAX_INTEGRAL = 0; // No max
    private static final double TALON_MAX_CLOSED_LOOP_VOLTAGE = 1.0; // volts
    private static final int TALON_CLOSED_LOOP_PERIOD = 1; // ms
    private static final boolean TALON_AUX_PID_POLARITY = false;
    private static final int TALON_MOTION_CRUISE_VELOCITY = 0; // Sensor ticks per 100ms
    private static final int TALON_MOTION_ACCElERATION = 0; // Sensor ticks per 100ms^2
    private static final int TALON_TRAJECTORY_PERIOD = 0; // ms
    private static final int TALON_PEAK_CURRENT_LIMIT = 60; // Amps before limiting kicks in
    private static final int TALON_PEAK_CURRENT_DURATION = 1; // Ms until limiting kicks in
    private static final int TALON_CONTINUOUS_CURRENT_LIMIT = 40; // Amps to limit at

    // Get the singleton instance of the drive
    public static Drive getInstance() {
        // Lazy initialization, to avoid early use of hardware resources during
        // class loading
        if (Objects.isNull(instance)) {
            instance = new Drive();
        }

        return instance;
    }
    

    // Initializes the drive
    private Drive() {
        
        // Instantiate master talon
        leftMasterTalon = new TalonSRX(LEFT_MASTER_CAN_ID);
        rightMasterTalon = new TalonSRX(RIGHT_MASTER_CAN_ID);
        
        // Configure the talons
        configDriveTalon(leftMasterTalon);
        configDriveTalon(rightMasterTalon);
        
        // Initialize followers talons
        for (int i = 0; i < MOTORS_PER_SIDE - 1; i++) {
            leftFollowerVictors[i] = new VictorSPX(LEFT_FOLLOWER_CAN_IDS[i]);
            leftFollowerVictors[i].follow(leftMasterTalon);
            leftFollowerVictors[i].setNeutralMode(NeutralMode.Brake);
            rightFollowerVictors[i] = new VictorSPX(RIGHT_FOLLOWER_CAN_IDS[i]);
            rightFollowerVictors[i].follow(rightMasterTalon);
            rightFollowerVictors[i].setNeutralMode(NeutralMode.Brake);
        }
        
        leftMasterTalon.setNeutralMode(NeutralMode.Brake);
        rightMasterTalon.setNeutralMode(NeutralMode.Brake);
    }

    // This is the open loop drive code that we determined to work best.
    public void bestOpenLoopDrive(double throttle, double turn) {
        // Apply exponential scaling (does not change range of values, just
        // makes more of the joystick correspond to lower speeds
        int throttleExponent = 3;
        int turnExponent = 3;
        
        if (Math.abs(throttle) < 0.001) {
        	throttle = throttle * .92;
        	Util.signedPow(throttleExponent, throttleExponent);
        	throttle += .08 * Math.signum(throttle);
        	
        	if (Math.abs(turn) < 0.001) {
            	turn = turn * .92;
            	Util.signedPow(turnExponent, turnExponent);
            	turn += .08 * Math.signum(turn);
            }
        } else if (Math.abs(turn) < 0.001) {
        	turn = turn * .92;
        	Util.signedPow(turnExponent, turnExponent);
        	turn += .075 * Math.signum(turn);
        }
        

        // Use cheesy drive
        turn = cheesifyTurn(throttle, turn);
        
        // Negative inertia!
        turn = negativeInertia(throttle, turn);
        

        basicArcade(throttle, turn);
    }

    // Negative inertia! Basically, the idea is that the robot has some inertia
    // which theoretically is based on previously commanded values. Returns an
    // updated turn value
    double mPrevTurn = 0; // The last turn value
    double mNegInertialAccumulator = 0; // Accumulates our current inertia value
    public double negativeInertia(double throttle, double turn) {
        // Constants for negative inertia
        final double LARGE_TURN_RATE_THRESHOLD = 0.65;
        final double INCREASE_TURN_SCALAR = 3.5;
        final double SMALL_DECREASE_TURN_SCALAR = 4.0;
        final double LARGE_DECREASE_TURN_SCALAR = 5.0;

        // How much we are currently trying to change the turn value
        double turnDiff = turn - mPrevTurn;
        mPrevTurn = turn;

        // Determine which scaling constant to use based on how we are moving
        double negInertiaScalar;
        if (turn * turnDiff > 0) {
            // We are trying to increase our turning rate
            negInertiaScalar = INCREASE_TURN_SCALAR;
        } else {
            if (Math.abs(turn) < LARGE_TURN_RATE_THRESHOLD) {
                // We are trying to reduce our turning rate to something
                // relatively close to 0
                negInertiaScalar = SMALL_DECREASE_TURN_SCALAR;
            } else {
                // We are trying to reduce our turning rate, but still want to
                // be turning fairly fast
                negInertiaScalar = LARGE_DECREASE_TURN_SCALAR;
            }
        }

        // Apply the scalar, and add it to the accumulator
        double negInertiaPower = turnDiff * negInertiaScalar;
        mNegInertialAccumulator += negInertiaPower;

        // Add the current negative inertia value to the turn
        double updatedTurn = turn + mNegInertialAccumulator;

        // Reduce our current inertia
        if (mNegInertialAccumulator > 1) {
            mNegInertialAccumulator -= 1;
        } else if (mNegInertialAccumulator < -1) {
            mNegInertialAccumulator += 1;
        } else {
            mNegInertialAccumulator = 0;
        }

        return updatedTurn;
    }

    // Scales the turn value based on the throttle so that a given turn value
    // corresponds to a set turning radius. Doesn't scale the value if throttle
    // is at 0. Returns an updated turn value
    public double cheesifyTurn(double throttle, double turn) {
        // A small error tolerance for floating point errors
        final double EPSILON = .001;

        // A threshold for the throttle where if it is too low the turn will be
        // use this minimum value (to stop turning from being too weak to
        // overcome static friction at low speeds)
        final double MIN_THROTTLE_TO_SCALE_TURN = .2;

        if (Math.abs(throttle) < EPSILON) {
            return turn;
        } else if (Math.abs(throttle) < MIN_THROTTLE_TO_SCALE_TURN) {
            return Math.signum(throttle) * MIN_THROTTLE_TO_SCALE_TURN * turn;
        } else {
            return throttle * turn;
        }
    }

    // Arcade drive control on open loop with no bells and whistles
    // Honestly pretty terrible. Turning precisely is basically impossibe
    public void basicArcade(double throttle, double turn) {

        double left = throttle + turn;
        double right = throttle - turn;

        openLoopDrive(left, right);
    }

    // Sets the raw values (from -1 to 1) for the left and right sides of the
    // drivetrain
    public void openLoopDrive(double leftPercent, double rightPercent) {
        // LEFT IS INVERTED HERE
        leftPercent = -leftPercent;
 
        leftMasterTalon.set(ControlMode.PercentOutput, leftPercent);
        rightMasterTalon.set(ControlMode.PercentOutput, rightPercent);
    }

    // Configures all of the persistent settings on a master talon
    private void configDriveTalon(TalonSRX talon) {
    	talon.enableCurrentLimit(true);
        
        talon.configSelectedFeedbackSensor(TALON_FEEDBACK_SENSOR, TALON_PID_IDX, TALON_CONFIG_TIMEOUT);
        talon.config_kP(TALON_PID_IDX, TALON_KP, TALON_CONFIG_TIMEOUT);
        talon.config_kI(TALON_PID_IDX, TALON_KI, TALON_CONFIG_TIMEOUT);
        talon.config_kD(TALON_PID_IDX, TALON_KD, TALON_CONFIG_TIMEOUT);
        talon.config_kF(TALON_PID_IDX, TALON_KF, TALON_CONFIG_TIMEOUT);

        talon.configOpenloopRamp(TALON_OPEN_LOOP_RAMP, TALON_CONFIG_TIMEOUT);
        talon.configClosedloopRamp(TALON_CLOSED_LOOP_RAMP, TALON_CONFIG_TIMEOUT);
        talon.configPeakOutputForward(TALON_MAX_FORWARD_VOLTAGE, TALON_CONFIG_TIMEOUT);
        talon.configPeakOutputReverse(TALON_MAX_REVERSE_VOLTAGE, TALON_CONFIG_TIMEOUT);
        talon.configNominalOutputForward(TALON_MIN_FORWARD_VOLTAGE, TALON_CONFIG_TIMEOUT);
        talon.configNominalOutputReverse(TALON_MIN_REVERSE_VOLTAGE, TALON_CONFIG_TIMEOUT);
        talon.configNeutralDeadband(TALON_NEUTRAL_DEADBAND, TALON_CONFIG_TIMEOUT);
        talon.configVoltageCompSaturation(TALON_VOLTAGE_COMP_SATURATION, TALON_CONFIG_TIMEOUT);
        talon.configVoltageMeasurementFilter(TALON_VOLTAGE_FILTER_WINDOW, TALON_CONFIG_TIMEOUT);
//        talon.configSelectedFeedbackCoefficient(TALON_FEEDBACK_COEFFICIENT, TALON_PID_IDX, TALON_CONFIG_TIMEOUT);
        talon.configVelocityMeasurementPeriod(TALON_VELOCITY_PERIOD, TALON_CONFIG_TIMEOUT);
        talon.configForwardLimitSwitchSource(TALON_LIMIT_SWITCH_SOURCE, TALON_LIMIT_SWITCH_NORMAL, TALON_CONFIG_TIMEOUT);
        talon.configReverseLimitSwitchSource(TALON_LIMIT_SWITCH_SOURCE, TALON_LIMIT_SWITCH_NORMAL, TALON_CONFIG_TIMEOUT);
        talon.configForwardSoftLimitThreshold(TALON_FORWARD_SOFT_LIMIT, TALON_CONFIG_TIMEOUT);
        talon.configReverseSoftLimitThreshold(TALON_REVERSE_SOFT_LIMIT, TALON_CONFIG_TIMEOUT);
        talon.configForwardSoftLimitEnable(TALON_FORWARD_SOFT_LIMIT_ENABLE, TALON_CONFIG_TIMEOUT);
        talon.configReverseSoftLimitEnable(TALON_REVERSE_SOFT_LIMIT_ENABLE, TALON_CONFIG_TIMEOUT);
        talon.config_IntegralZone(TALON_PID_IDX, TALON_INTEGRAL_ZONE, TALON_CONFIG_TIMEOUT);
        talon.configAllowableClosedloopError(TALON_PID_IDX, TALON_MIN_CLOSED_LOOP_ERROR, TALON_CONFIG_TIMEOUT);
        talon.configMaxIntegralAccumulator(TALON_PID_IDX, TALON_MAX_INTEGRAL, TALON_CONFIG_TIMEOUT);
//        talon.configClosedLoopPeakOutput(TALON_PID_IDX, TALON_MAX_CLOSED_LOOP_VOLTAGE, TALON_CONFIG_TIMEOUT);
//        talon.configClosedLoopPeriod(TALON_PID_IDX, TALON_CLOSED_LOOP_PERIOD, TALON_CONFIG_TIMEOUT);
//        talon.configAuxPIDPolarity(TALON_AUX_PID_POLARITY, TALON_CONFIG_TIMEOUT);
        talon.configMotionCruiseVelocity(TALON_MOTION_CRUISE_VELOCITY, TALON_CONFIG_TIMEOUT);
        talon.configMotionAcceleration(TALON_MOTION_ACCElERATION, TALON_CONFIG_TIMEOUT);
        talon.configMotionProfileTrajectoryPeriod(TALON_TRAJECTORY_PERIOD, TALON_CONFIG_TIMEOUT);
        talon.configPeakCurrentLimit(TALON_PEAK_CURRENT_LIMIT, TALON_CONFIG_TIMEOUT);
        talon.configContinuousCurrentLimit(TALON_CONTINUOUS_CURRENT_LIMIT, TALON_CONFIG_TIMEOUT);
    }
        
}
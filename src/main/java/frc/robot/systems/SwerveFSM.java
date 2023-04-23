package frc.robot.systems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class SwerveFSM extends SubsystemBase{
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		DRIVE
	}

    private static final double TRACK_WIDTH = Units.inchesToMeters(24); //PLACEHOLDER
    private static final double WHEEL_BASE = Units.inchesToMeters(24); //PLACEHOLDER
    public static SwerveDriveKinematics driveKinematics =  new SwerveDriveKinematics(
		new Translation2d(WHEEL_BASE/2, -TRACK_WIDTH/2),
		new Translation2d(WHEEL_BASE/2, TRACK_WIDTH/2),
		new Translation2d(-WHEEL_BASE/2, -TRACK_WIDTH/2),
		new Translation2d(-WHEEL_BASE/2, TRACK_WIDTH/2)
	);;

	private static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = true;
    private static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = true;
	private static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = true;
    private static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = true;

    private static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = true;
    private static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = true;
    private static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false;
    private static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false;

    private static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    private static final boolean FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    private static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    private static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;

    private static final CANCoder FRONT_LEFT_CANCODER = new CANCoder(HardwareMap.FRONT_LEFT_CANCODER_ID);
    private static final CANCoder FRONT_RIGHT_CANCODER = new CANCoder(HardwareMap.FRONT_RIGHT_CANCODER_ID);
    private static final CANCoder BACK_LEFT_CANCODER = new CANCoder(HardwareMap.BACK_LEFT_CANCODER_ID);
    private static final CANCoder BACK_RIGHT_CANCODER = new CANCoder(HardwareMap.BACK_RIGHT_CANCODER_ID);

    private static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0; //PLACEHOLDER VALUE
    private static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0; //PLACEHOLDER VALUE
    private static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0; //PLACEHOLDER VALUE
    private static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0; //PLACEHOLDER VALUE
    //NEED TO FIND ACTUAL VALUES USING PHOENIX TUNER ONCE WE GET THE ACTUAL MECH

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    //constants
    private static final double DRIVE_MAX_SPEED_MPS = SwerveModule.getPhysicalMaxSpeed() / 4.0; //adjust based on speed, capped to maintain controllability
    private static final double DRIVE_MAX_ANGULAR_SPEED_RPS = 4 * Math.PI;
    private static final double DRIVE_MAX_SPEED_ACCELERATION = 3;
    private static final double DRIVE_MAX_ANGULAR_ACCELERATION = 3;
    private static final double AXIS_DEADBAND = 0.025;

    private SlewRateLimiter xDriveLimiter;
    private SlewRateLimiter yDriveLimiter;
    private SlewRateLimiter turningLimiter;

	private final SwerveDriveOdometry odometer;

    private final AHRS gyro;

    
	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public SwerveFSM() {
		// Perform hardware init
		frontLeft = new SwerveModule(HardwareMap.CAN_ID_FRONT_LEFT_DRIVE, HardwareMap.CAN_ID_FRONT_LEFT_TURN,
            FRONT_LEFT_DRIVE_ENCODER_REVERSED, FRONT_LEFT_TURNING_ENCODER_REVERSED, FRONT_LEFT_CANCODER,
            FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED, FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
        frontRight = new SwerveModule(HardwareMap.CAN_ID_FRONT_RIGHT_DRIVE, HardwareMap.CAN_ID_FRONT_RIGHT_TURN,
            FRONT_RIGHT_DRIVE_ENCODER_REVERSED, FRONT_RIGHT_TURNING_ENCODER_REVERSED, FRONT_RIGHT_CANCODER,
            FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED, FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
        backLeft = new SwerveModule(HardwareMap.CAN_ID_BACK_LEFT_DRIVE, HardwareMap.CAN_ID_BACK_LEFT_TURN,
            BACK_LEFT_DRIVE_ENCODER_REVERSED, BACK_LEFT_TURNING_ENCODER_REVERSED, BACK_LEFT_CANCODER,
            BACK_LEFT_ABSOLUTE_ENCODER_REVERSED, BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
        backRight = new SwerveModule(HardwareMap.CAN_ID_BACK_RIGHT_DRIVE, HardwareMap.CAN_ID_BACK_RIGHT_TURN,
            BACK_RIGHT_DRIVE_ENCODER_REVERSED, BACK_RIGHT_TURNING_ENCODER_REVERSED, BACK_RIGHT_CANCODER,
            BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED, BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS);

        xDriveLimiter = new SlewRateLimiter(DRIVE_MAX_SPEED_ACCELERATION);
        yDriveLimiter = new SlewRateLimiter(DRIVE_MAX_SPEED_ACCELERATION);
        turningLimiter = new SlewRateLimiter(DRIVE_MAX_ANGULAR_ACCELERATION);

		odometer = new SwerveDriveOdometry(driveKinematics, new Rotation2d(0), getSwerveModulePositions());

        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.DRIVE;
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	public void updateAutonomous() {
		odometer.update(Rotation2d.fromDegrees(gyro.getAngle()), getSwerveModulePositions());
	}


	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {

		odometer.update(Rotation2d.fromDegrees(gyro.getAngle()), getSwerveModulePositions());
		
		switch (currentState) {
			case DRIVE:
				handleDriveState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case DRIVE:
				return SwerveFSM.FSMState.DRIVE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleDriveState(TeleopInput input) {
        if(input==null) return;

        double xAxis = input.getSwerveJoystickLeftX();
        double yAxis = input.getSwerveJoystickLeftY(); //already negated in teleopinput
        double turnAxis = input.getSwerveJoystickRightX();

        xAxis = (Math.abs(xAxis) > AXIS_DEADBAND) ? xAxis : 0;
        yAxis = (Math.abs(yAxis) > AXIS_DEADBAND) ? yAxis : 0;
        turnAxis = (Math.abs(turnAxis) > AXIS_DEADBAND) ? turnAxis : 0;

		xAxis = xDriveLimiter.calculate(xAxis) * DRIVE_MAX_SPEED_MPS;
        yAxis = yDriveLimiter.calculate(yAxis) * DRIVE_MAX_SPEED_MPS;
        turnAxis = turningLimiter.calculate(turnAxis) * DRIVE_MAX_ANGULAR_SPEED_RPS;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xAxis, yAxis, turnAxis, getHeading());

        SwerveModuleState[] states = driveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
	}

    public void setModuleStates(SwerveModuleState[] targets) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targets, SwerveModule.getPhysicalMaxSpeed());
        frontLeft.setState(targets[0]);
        frontRight.setState(targets[1]);
        backLeft.setState(targets[2]);
        backRight.setState(targets[3]);
    }

	public SwerveModuleState[] getSwerveModuleStates() {
		SwerveModuleState[] ret = new SwerveModuleState[4];
		ret[0] = frontLeft.getCurrentState();
		ret[1] = frontRight.getCurrentState();
		ret[2] = backLeft.getCurrentState();
		ret[3] = backRight.getCurrentState();
		return ret;
	}

	private SwerveModulePosition[] getSwerveModulePositions() {
		SwerveModulePosition[] ret = new SwerveModulePosition[4];
		ret[0] = frontLeft.getPosition();
		ret[1] = frontRight.getPosition();
		ret[2] = backLeft.getPosition();
		ret[3] = backRight.getPosition();
		return ret;
	}

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

	//THIS IS IN DEGREES NOT RADIANS
    private Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360));
    }

	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		odometer.resetPosition(getHeading(), getSwerveModulePositions(), pose);
	}
}

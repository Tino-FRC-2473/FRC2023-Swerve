package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int SWERVE_CONTROLLER_PORT = 0;
	private static final int RIGHT_JOYSTICK_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private XboxController swerveController; //logitech gamepad f310, must be set to XInput mode on back
	private Joystick rightJoystick;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		swerveController = new XboxController(SWERVE_CONTROLLER_PORT);
		rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Get X axis of left joystick of Swerve controller.
	 * @return Axis value
	 */
	public double getSwerveJoystickLeftX() {
		return swerveController.getLeftX();
	}
	/**
	 * Get Y axis of left joystick of Swerve controller.
	 * @return Axis value
	 */
	public double getSwerveJoystickLeftY() {
		return -swerveController.getLeftY(); //may or may not need the -, testing needed
	}
	/**
	 * Get X axis of right joystick of Swerve controller.
	 * @return Axis value
	 */
	public double getSwerveJoystickRightX() {
		return swerveController.getRightX();
	}
	

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return rightJoystick.getX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}

	/* ======================== Private methods ======================== */

}

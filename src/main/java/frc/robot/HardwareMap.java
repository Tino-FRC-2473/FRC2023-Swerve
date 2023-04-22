package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	//swerve module CAN ID's
	public static final int CAN_ID_FRONT_LEFT_DRIVE = 0; //THIS IS A PLACEHOLDER
	public static final int CAN_ID_FRONT_LEFT_TURN = 1; //THIS IS A PLACEHOLDER
	public static final int CAN_ID_FRONT_RIGHT_DRIVE = 2; //THIS IS A PLACEHOLDER
	public static final int CAN_ID_FRONT_RIGHT_TURN = 3; //THIS IS A PLACEHOLDER
	public static final int CAN_ID_BACK_LEFT_DRIVE = 4; //THIS IS A PLACEHOLDER
	public static final int CAN_ID_BACK_LEFT_TURN = 5; //THIS IS A PLACEHOLDER
	public static final int CAN_ID_BACK_RIGHT_DRIVE = 6; //THIS IS A PLACEHOLDER
	public static final int CAN_ID_BACK_RIGHT_TURN = 7; //THIS IS A PLACEHOLDER

	//CANCoder port ID's
	public static final int FRONT_LEFT_CANCODER_ID = 0; //THIS IS A PLACEHOLDER
	public static final int FRONT_RIGHT_CANCODER_ID= 1; //THIS IS A PLACEHOLDER
	public static final int BACK_LEFT_CANCODER_ID = 2; //THIS IS A PLACEHOLDER
	public static final int BACK_RIGHT_CANCODER_ID = 3; //THIS IS A PLACEHOLDER

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL = 9;
	private static DigitalInput testBoardPin = new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL);
	/**
	 * Check if the current RoboRIO is part of a test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoard() {
		return !HardwareMap.testBoardPin.get();
	}
}

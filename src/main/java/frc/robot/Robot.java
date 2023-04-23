// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.systems.AutonomousTrajectoryChooser;
// Systems
import frc.robot.systems.FSMSystem;
import frc.robot.systems.SwerveFSM;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	private SwerveFSM swerve;
	private AutonomousTrajectoryChooser chooser;
	SequentialCommandGroup autoCommand;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		// Instantiate all systems here
		swerve = new SwerveFSM();
		chooser = new AutonomousTrajectoryChooser();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		swerve.reset();

		PIDController xController = new PIDController(1.5, 0, 0);
        PIDController yController = new PIDController(1.5, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                3, 0, 0, new TrapezoidProfile.Constraints(
					Math.PI,
					Math.PI/4));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

		Trajectory trajectory = chooser.getSelectedTrajectory();

		SwerveControllerCommand commander = new SwerveControllerCommand(
			trajectory,
			swerve::getPose,
			SwerveFSM.driveKinematics,
			xController,
			yController,
			thetaController,
			swerve::setModuleStates,
			swerve);

		autoCommand = new SequentialCommandGroup(
			new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
			commander,
			new InstantCommand(() -> swerve.stop())
		);

		autoCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {
		swerve.updateAutonomous();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		if(autoCommand != null) {
			autoCommand.cancel();
		}
		swerve.reset();
	}

	@Override
	public void teleopPeriodic() {
		swerve.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}

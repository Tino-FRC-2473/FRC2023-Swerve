package frc.robot.systems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public final class AutonomousTrajectories {

    private static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3,3)
    .setKinematics(SwerveFSM.driveKinematics);

    public static final class Trajectories {
        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(2, 1),
                        new Translation2d(0, 0),
                        new Translation2d(0, 1),
                        new Translation2d(-1, -2)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(90)),
                trajectoryConfig);
    }
}

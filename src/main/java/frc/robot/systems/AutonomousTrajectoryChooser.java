package frc.robot.systems;

import java.io.IOException;
import java.nio.file.Path;
import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousTrajectoryChooser {

    private SendableChooser<Trajectory> trajectoryChooser;

    public AutonomousTrajectoryChooser() {
        trajectoryChooser = new SendableChooser<>();
        trajectoryChooser.setDefaultOption("Straight Path", jsonToSwerveTrajectory("src/main/deploy/deploy/pathplanner/generatedJSON/straight_path.wpilib.json"));
        trajectoryChooser.addOption("Curved Path", jsonToSwerveTrajectory("src/main/deploy/deploy/pathplanner/generatedJSON/curved_path.wpilib.json"));
        SmartDashboard.putData("Trajectory Path", trajectoryChooser);
    }

    private Trajectory jsonToSwerveTrajectory(String filename) {
        Trajectory trajectory;

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch(IOException exception) {
            DriverStation.reportError("Unable to create trajectory from path", exception.getStackTrace());
            System.out.println("Unable to create path from " + filename);
            return null;
        }

        return trajectory;
    }

    public SendableChooser getTrajectoryChooser() {
        return trajectoryChooser;
    }

    public Trajectory getSelectedTrajectory() {
        return trajectoryChooser.getSelected();
    }
}

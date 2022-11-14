package frc.robot;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

public class Auto {
    Timer timer = new Timer();

    public void LoadTrajectory(String name){

        timer.reset();
        timer.start();

        String trajectoryJSON = "paths/New Path.wpilib.json";
        Trajectory trajectory = new Trajectory();

        try {
            Path deployDirectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(deployDirectory);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
         }

        ahrs.Reset(double(trajectory.InitialPose().Rotation().Degrees()));

    }
}

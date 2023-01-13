package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
// import org.photonvision.RobotPoseEstimator;
// import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class FiducalCamera {
    public FiducalCamera(){
    }
    
    // Photon photon;
    // RobotPoseEstimator robotPoseEstimator;
    PhotonCamera camera = new PhotonCamera("GlobalShutterCamera");
    PhotonPipelineResult result;

    double getFiducalYaw(){
        result = camera.getLatestResult();
        if( result.hasTargets()){
            // System.out.println("in target: " + result.getBestTarget().getYaw());
            return result.getBestTarget().getYaw();
        }
        // System.out.println("no target");

        return 0;
    }



}

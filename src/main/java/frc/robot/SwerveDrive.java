package frc.robot;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.AHRSProtocol.AHRS_DATA_ACTION;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

public class SwerveDrive{

    // status variable for being enabled
    public boolean mIsEnabled = false;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveMod[] SwerveMods;
    public double headingSetPoint;
    private PIDController controller = new PIDController(0.7,0,0);

    private PIDController xController = new PIDController(0.7,0,0);
    private PIDController yController = new PIDController(0.7,0,0);
    // private PIDController thetaController = new PIDController(0.25,0.1,0);
    private PIDController thetaController = new PIDController(0.05,0,0);
    private ProfiledPIDController thetaController2 = new ProfiledPIDController(0.5,0,0, new TrapezoidProfile.Constraints(40, 180));
    public HolonomicDriveController autoController = new HolonomicDriveController(xController, yController, thetaController2);
    public Trajectory trajectory = new Trajectory();
    public Trajectory turningTrajectory = new Trajectory();
    public Timer timer = new Timer();
    public String File = "pathplanner/generatedJSON/CorrectPath.path"; 
    public String Json;
    public int array[] = {};   
    // public Path deployDirectory = new Path();


    AHRS ahrs = new AHRS(SPI.Port.kMXP);

    // chassis velocity status
    ChassisSpeeds chassisVelocity = new ChassisSpeeds();

    // public boolean isSnapping;

    // public ProfiledPIDController snapPIDController;
    
    
    // Private boolean to lock Swerve wheels
    private boolean mLocked = false;

    // Getter
    public boolean getLocked() {
        return mLocked;
    }
    // Setter
    public void setLocked(boolean lock) {
        mLocked = lock;
    }

    public SwerveDrive() {        
        swerveOdometry = new SwerveDriveOdometry(Settings.SwerveConstants.swerveKinematics, Rotation2d.fromDegrees(-ahrs.getYaw()));
        
        // snapPIDController = new ProfiledPIDController(Constants.SnapConstants.kP,
        //                                               Constants.SnapConstants.kI, 
        //                                               Constants.SnapConstants.kD,
        //                                               Constants.SnapConstants.kThetaControllerConstraints);
        // snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

        zeroGyro();
        SwerveMods = new SwerveMod[] {
            //MODULE 0 AND 3 MIGHT BE SLIGHTLY OFF
            new SwerveMod(0, 4, 8, 9, 254.3 - 1.23 + 2.813),
            new SwerveMod(1, 1, 5, 12, 121.4 - 0.88),
            new SwerveMod(2, 3, 2, 11, 33.8 + 0.09),
            new SwerveMod(3, 6, 7, 10, 44.5  + 2.37 - 5.274),
        };
    }
/*
    // system.out.println("X " + swerveOdometry.getX);
    // system.out.println("Y " + swerveOdometry.getY);

    // @Override
    // public void registerEnabledLoops(ILooper mEnabledLooper) {
    //     mEnabledLooper.register(new Loop() {
    //         @Override
    //         public void onStart(double timestamp) {
    //             mIsEnabled = true;
    //         }

    //         @Override
    //         public void onLoop(double timestamp) {
    //             mIsEnabled = false;
    //             chooseVisionAlignGoal();
    //             updateSwerveOdometry();
    //         }

    //         @Override
    //         public void onStop(double timestamp) {
    //             stop();
    //         }
    //     });
    // }

    // public void angleAlignDrive(Translation2d translation2d, double targetHeading, boolean fieldRelative) {
    //     snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(targetHeading), 0.0));
    //     double angleAdjustment = snapPIDController.calculate(-ahrs.getYaw());
    //     drive(translation2d, angleAdjustment, fieldRelative, false);
    // }

    */

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        System.out.println("X: " + swerveOdometry.getPoseMeters().getX());
        System.out.println("Y: " + swerveOdometry.getPoseMeters().getY());
        System.out.println("Degrees: " + swerveOdometry.getPoseMeters().getRotation().getDegrees());
        System.out.println("Yaw: " + ahrs.getYaw());
        headingSetPoint += rotation * 0.24;
        // if(headingSetPoint > 179){
        //     headingSetPoint -= 360;
        // }
        // if(headingSetPoint < -179){
        //     headingSetPoint += 360;
        // }
        // System.out.println("heading " + headingSetPoint);
        // System.out.println("Navx " + -ahrs.getAngle());
        // SmartDashboard.getNumber("Heading Set Point", headingSetPoint);
        // SmartDashboard.getNumber("Navx", -ahrs.getAngle());

        // if (isSnapping) {
        //     if (Math.abs(rotation) == 0.0) {
        //         maybeStopSnap(false);
        //         rotation = calculateSnapValue();
        //     } else {
        //         maybeStopSnap(true);
        //     }
        // }
       
        SwerveModuleState[] swerveModuleStates = null;
        if (mLocked) {
            swerveModuleStates = new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0))
            };
        } else {
            swerveModuleStates =
                Settings.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), 
                                        translation.getY(), 
                                        controller.calculate(ahrs.getAngle(), headingSetPoint), 
                                        Rotation2d.fromDegrees(ahrs.getYaw())
                                    )
                                    : new ChassisSpeeds(
                                        translation.getX(), 
                                        translation.getY(),
                                        rotation)
                                    );
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Settings.SwerveConstants.maxSpeed);

        for (SwerveMod mod : SwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
            // mod.setDesiredState(new SwerveModuleState(0, mod.getState().angle));
        }
    }

/*
//     public void zeroHeading() {
//     m_ahrs.zeroYaw();
//     offset = 0;
//     m_targetPose = new Pose2d(new Translation2d(), new Rotation2d());
//     }

//      public Rotation2d getHeading() {
//     float raw_yaw = m_-ahrs.getYaw() - (float)offset; // Returns yaw as -180 to +180.
//     // float raw_yaw = m_-ahrs.getYaw(); // Returns yaw as -180 to +180.
//     float calc_yaw = raw_yaw;

//     if (0.0 > raw_yaw ) { // yaw is negativez
//       calc_yaw += 360.0;
//     }
//     return Rotation2d.fromDegrees(-calc_yaw);
//   }
// }


    // public double calculateSnapValue() {
    //     return snapPIDController.calculate(mPigeon.getYaw().getRadians());
    // }

    // public void startSnap(double snapAngle) {
    //     snapPIDController.reset(mPigeon.getYaw().getRadians());
    //     snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
    //     isSnapping = true;
    // }
    
    // TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    // private boolean snapComplete() {
    //     double error = snapPIDController.getGoal().position - mPigeon.getYaw().getRadians();
    //     return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.kEpsilon), Constants.SnapConstants.kTimeout);
    // }

    // public void maybeStopSnap(boolean force){
    //     if (!isSnapping) {
    //         return;
    //     } 
    //     if (force || snapComplete()) {
    //         isSnapping = false;
    //         snapPIDController.reset(mPigeon.getYaw().getRadians());
    //     }
    // }

    // //Used by SwerveControllerCommand in Auto
    // public void setModuleStates(SwerveModuleState[] desiredStates) {
    //     SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
    //     for(SwerveMod mod : SwerveMods){
    //         mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    //         SmartDashboard.putNumber("mod " + mod.moduleNumber +  " desired speed", desiredStates[mod.moduleNumber].speedMetersPerSecond);
    //         SmartDashboard.putNumber("mod " + mod.moduleNumber +  " desired angle", MathUtil.inputModulus(desiredStates[mod.moduleNumber].angle.getDegrees(), 0, 180));
    //     }
    // }    
    */
        
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }
    // public Pose2d getPose() {
    //     return new Pose2d(swerveOdometry.getPoseMeters().getX(),swerveOdometry.getPoseMeters().getY(),new Rotation2d(0));
    // }
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, Rotation2d.fromDegrees(headingSetPoint));
        // zeroGyro(pose.getRotation().getDegrees());


    }

    public void resetAnglesToAbsolute() {
        for (SwerveMod mod : SwerveMods) {
            // mod.resetToAbsolute();
            mod.syncEncoders();
            mod.resetDrive();
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveMod mod : SwerveMods){
            states[mod.moduleNumber] = mod.getState();
            SmartDashboard.putNumber("mod " + mod.moduleNumber + " current speed", states[mod.moduleNumber].speedMetersPerSecond);
            SmartDashboard.putNumber("mod " + mod.moduleNumber + " current angle", MathUtil.inputModulus(states[mod.moduleNumber].angle.getDegrees(), 0, 180));
        }
        return states;
    }

    // public void setAnglePIDValues(double kP, double kI, double kD) {
    //     for (SwerveModule swerveModule : mSwerveMods) {
    //         swerveModule.updateAnglePID(kP, kI, kD);
    //     }
    // }

    // public double[] getAnglePIDValues(int index) {
    //     return mSwerveMods[index].getAnglePIDValues();
    // }
    
    public void zeroGyro(){
        ahrs.reset();
    }

    public void updateSwerveOdometry(){
        swerveOdometry.update(Rotation2d.fromDegrees(-ahrs.getYaw()), getStates());
        chassisVelocity = Settings.SwerveConstants.swerveKinematics.toChassisSpeeds(
                    SwerveMods[0].getState(),
                    SwerveMods[1].getState(),
                    SwerveMods[2].getState(),
                    SwerveMods[3].getState()
            );
    }

    public void loadTrajectory(String name){
        String trajectoryJSON = "pathplanner/generatedJSON/" + name;
        Path deployDirectory;
        try {
            deployDirectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(deployDirectory);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        // Json = deployDirectory.toString();


        zeroGyro();
        ahrs.setAngleAdjustment(0);
        // ahrs.setAngleAdjustment(0);
         //                                                       The robot fields angle (in pathweaver rotation)
        System.out.println("trajectory: " +trajectory.getInitialPose().getRotation());
        //Rotation2d.fromDegrees(90)
        swerveOdometry.resetPosition(trajectory.getInitialPose(), trajectory.getInitialPose().getRotation());
        timer.reset();
        timer.start();
    }

    public boolean followTrajectory(double holonomicRotation){
     
        updateSwerveOdometry();
        Trajectory.State goal = trajectory.sample(timer.get());
                    //                                                          rotation in Path Weaver
        ChassisSpeeds adjustedSpeeds = autoController.calculate(getPose(), goal, Rotation2d.fromDegrees(0));
        System.out.println("X real: "+ getPose().getX());
        System.out.println("Y real: "+ getPose().getY());

        System.out.println("X goal: "+ goal.poseMeters.getX());
        System.out.println("Y goal: "+ goal.poseMeters.getY());

        System.out.println("Degrees: " + swerveOdometry.getPoseMeters().getRotation().getDegrees());
        System.out.println("Yaw: " + ahrs.getYaw());

        adjustedSpeeds.vyMetersPerSecond *=-1;
        // adjustedSpeeds.vxMetersPerSecond *=-1;

        System.out.println("Y Vel: " + adjustedSpeeds.vyMetersPerSecond);
        System.out.println("X Vel: " + adjustedSpeeds.vxMetersPerSecond);

        // adjustedSpeeds.omegaRadiansPerSecond = thetaController.calculate(ahrs.getYaw(), 90);
        // ChassisSpeeds adjustedSpeeds2 = new ChassisSpeeds(0, 0, thetaController.calculate(ahrs.getYaw(), 90));
        ChassisSpeeds adjustedSpeeds2 = new ChassisSpeeds(adjustedSpeeds.vxMetersPerSecond,adjustedSpeeds.vyMetersPerSecond, thetaController.calculate(ahrs.getYaw(), holonomicRotation));

        // SwerveModuleState[] swerveModuleStates =
        //         Settings.SwerveConstants.swerveKinematics.toSwerveModuleStates(
        //             true ? ChassisSpeeds.fromFieldRelativeSpeeds(
        //                                 goal.velocityMetersPerSecond*goal.poseMeters.getRotation().getCos(), 
        //                                 goal.velocityMetersPerSecond*goal.poseMeters.getRotation().getSin(), 
        //                                 1, 
        //                                 Rotation2d.fromDegrees(-ahrs.getYaw())
        //                             )
        //                             : new ChassisSpeeds(
        //                                 adjustedSpeeds.vxMetersPerSecond, 
        //                                 adjustedSpeeds.vyMetersPerSecond,
        //                                 0.5)
        //                             );
        SwerveModuleState[] swerveModuleStates = Settings.SwerveConstants.swerveKinematics.toSwerveModuleStates(adjustedSpeeds2);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Settings.SwerveConstants.maxSpeed);

        for (SwerveMod mod : SwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
        }
        System.out.println("Time: " +timer.get());
        // if (timer.get() > trajectory.getTotalTimeSeconds()){
        //     return true;
        // }
        if (timer.get() > 100){
            return true;
        }
        return false;
    }




}


package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive{

    // status variable for being enabled
    public boolean mIsEnabled = false;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveMod[] SwerveMods;


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
        swerveOdometry = new SwerveDriveOdometry(Settings.SwerveConstants.swerveKinematics, new Rotation2d(ahrs.getYaw()));
        
        // snapPIDController = new ProfiledPIDController(Constants.SnapConstants.kP,
        //                                               Constants.SnapConstants.kI, 
        //                                               Constants.SnapConstants.kD,
        //                                               Constants.SnapConstants.kThetaControllerConstraints);
        // snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

        zeroGyro();

        SwerveMods = new SwerveMod[] {
            new SwerveMod(0, 4, 8, 9, 257),
            new SwerveMod(1, 1, 5, 12, 120),
            new SwerveMod(2, 3, 2, 11, 32),
            new SwerveMod(3, 6, 7, 10, 45),
        };
    }

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
    //     double angleAdjustment = snapPIDController.calculate(ahrs.getYaw());
    //     drive(translation2d, angleAdjustment, fieldRelative, false);
    // }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
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
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(0))
            };
        } else {
            System.out.println(ahrs.getYaw());
            swerveModuleStates =
                Settings.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), 
                                        translation.getY(), 
                                        rotation, 
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


//     public void zeroHeading() {
//     m_ahrs.zeroYaw();
//     offset = 0;
//     m_targetPose = new Pose2d(new Translation2d(), new Rotation2d());
//     }

//      public Rotation2d getHeading() {
//     float raw_yaw = m_ahrs.getYaw() - (float)offset; // Returns yaw as -180 to +180.
//     // float raw_yaw = m_ahrs.getYaw(); // Returns yaw as -180 to +180.
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

    // /* Used by SwerveControllerCommand in Auto */
    // public void setModuleStates(SwerveModuleState[] desiredStates) {
    //     SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
    //     for(SwerveMod mod : SwerveMods){
    //         mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    //         SmartDashboard.putNumber("mod " + mod.moduleNumber +  " desired speed", desiredStates[mod.moduleNumber].speedMetersPerSecond);
    //         SmartDashboard.putNumber("mod " + mod.moduleNumber +  " desired angle", MathUtil.inputModulus(desiredStates[mod.moduleNumber].angle.getDegrees(), 0, 180));
    //     }
    // }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    // public void resetOdometry(Pose2d pose) {
    //     swerveOdometry.resetPosition(pose, pose.getRotation());
    //     zeroGyro(pose.getRotation().getDegrees());

    //     // reset field to vehicle
    //     RobotState.getInstance().reset(new com.team254.lib.geometry.Pose2d(pose));
    // }

    public void resetAnglesToAbsolute() {
        for (SwerveMod mod : SwerveMods) {
            // mod.resetToAbsolute();
            mod.syncEncoders();
        }
    }

    // public SwerveModuleState[] getStates() {
    //     SwerveModuleState[] states = new SwerveModuleState[4];
    //     for(SwerveMod mod : SwerveMods){
    //         states[mod.moduleNumber] = mod.getState();
    //         SmartDashboard.putNumber("mod " + mod.moduleNumber + " current speed", states[mod.moduleNumber].speedMetersPerSecond);
    //         SmartDashboard.putNumber("mod " + mod.moduleNumber + " current angle", MathUtil.inputModulus(states[mod.moduleNumber].angle.getDegrees(), 0, 180));
    //     }
    //     return states;
    // }

    // public void setAnglePIDValues(double kP, double kI, double kD) {
    //     for (SwerveModule swerveModule : mSwerveMods) {
    //         swerveModule.updateAnglePID(kP, kI, kD);
    //     }
    // }

    // public double[] getAnglePIDValues(int index) {
    //     return mSwerveMods[index].getAnglePIDValues();
    // }


    // @Override
    // public void zeroSensors(){
    //     zeroGyro(0.0);
    // }
    
    public void zeroGyro(){
        zeroGyro(0.0);
    }

    public void zeroGyro(double reset){
        ahrs.reset();
    }

    // public void updateSwerveOdometry(){
    //     swerveOdometry.update(mPigeon.getYaw().getWPIRotation2d(), getStates());

    //     chassisVelocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(
    //                 mInstance.mSwerveMods[0].getState(),
    //                 mInstance.mSwerveMods[1].getState(),
    //                 mInstance.mSwerveMods[2].getState(),
    //                 mInstance.mSwerveMods[3].getState()
    //         );
    // }

    // @Override
    // public void stop() {
    //     mIsEnabled = false;
    // }

    // @Override
    // public boolean checkSystem() {
    //     return true;
    // }

}
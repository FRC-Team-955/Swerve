// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SwerveDrive swerve;
  private FiducalCamera fiducalCamera;
    
  private ControlBoard controlBoard;
  int autoState = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    swerve = new SwerveDrive();
    fiducalCamera = new FiducalCamera();
    controlBoard = new ControlBoard();
    swerve.resetAnglesToAbsolute();
    swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
    autoState = 0;
  }
  @Override
  public void autonomousPeriodic() {
    if (true){
      if(autoState ==0){
        System.out.println("autostate0");
        swerve.loadTrajectory("New.wpilib.json");
        autoState++;
      }
      if(autoState ==1){
        System.out.println("autostate1");
        if(swerve.followTrajectory(180)){
          autoState++;
        }
      }
      if(autoState ==2){
        System.out.println("autostate2");
        swerve.loadTrajectory("2nd.wpilib.json");
        autoState++;
      }
      if(autoState ==3){
        System.out.println("autostate3");
        if(swerve.followTrajectory(0)){
          autoState++;
        }
      }
      if(autoState ==4){
        System.out.println("autostate4");
        swerve.drive(new Translation2d(0,0), 0, false, true);
      }
    }
  }

  @Override
  public void teleopInit() {
    swerve.setHeading();
  }

  @Override
  public void teleopPeriodic() {
    Translation2d swerveTranslation = new Translation2d(controlBoard.getSwerveTranslation().getX(), controlBoard.getSwerveTranslation().getY());
    // System.out.println(swerveTranslation);
    // System.out.println(swerveTranslation.getX());
    // System.out.println(swerveTranslation.getY());
    // Translation2d swerveTranslation = new Translation2d(1, 0);
    double swerveRotation = controlBoard.getSwerveRotation();
    // swerve.drive(swerveTranslation, swerveRotation, false, true);

    // fiducalCamera.getFiducalAngle();
    //fiducalCamera.getFiducalAngle()
    // System.out.println("camera: " + fiducalCamera.getFiducalYaw());
    // swerve.angleAlignDrive(swerveTranslation,fiducalCamera.getFiducalYaw(), false);
    swerve.translationAlignDrive(fiducalCamera.getFiducalYaw(),swerveRotation, false);
    // swerve.drive(swerveTranslation, fiducalCamera.getFiducalYaw(), false, true);

    swerve.updateSwerveOdometry();
  }

  @Override
  public void disabledInit() {}

  @Override 
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}

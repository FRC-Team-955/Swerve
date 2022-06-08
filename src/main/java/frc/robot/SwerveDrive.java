package frc.robot;


import java.io.Console;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.SwerveMod;

public class SwerveDrive {
    int robot_width = 10;
    double distance = robot_width/2;
    Translation2d m_frontLeftLocation = new Translation2d(distance, distance);
    Translation2d m_frontRightLocation = new Translation2d(distance, -distance);
    Translation2d m_backLeftLocation = new Translation2d(-distance, distance);
    Translation2d m_backRightLocation = new Translation2d(-distance, -distance);
  
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLocation,m_frontRightLocation,
                                                                  m_backLeftLocation,m_backRightLocation);
  
    Joystick joystick = new Joystick(0);

    CANCoder frontLeft_cancoder = new CANCoder(0);
    CANCoder frontRight_cancoder = new CANCoder(1);
    CANCoder backLeft_cancoder = new CANCoder(2);
    CANCoder backRight_cancoder = new CANCoder(3);
    
        public void drive(){
        }

    public void getState(){

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        
        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight  = moduleStates[1];
        SwerveModuleState backLeft  = moduleStates[2];
        SwerveModuleState backRight  = moduleStates[3];
        var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new Rotation2d(frontLeft_cancoder.getPosition()));
        var frontRightOptimized = SwerveModuleState.optimize(frontRight, new Rotation2d(frontRight_cancoder.getPosition()));
        var backLeftOptimized = SwerveModuleState.optimize(backLeft, new Rotation2d(backLeft_cancoder.getPosition()));
        var backRightLeftOptimized = SwerveModuleState.optimize(backRight, new Rotation2d(backRight_cancoder.getPosition()));
    }
}

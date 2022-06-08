package frc.robot;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
public class Settings{
    public static final class SwerveConstants {
        /* Angle Motor PID Values */
        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    public static  boolean canCoderInvert = false;
    public static double maxSpeed = 10;

    public static boolean angleMotorInvert = false;
    public static IdleMode angleIdleMode = IdleMode.kBrake;
    public static boolean driveMotorInvert = false;
    public static IdleMode driveIdleMode = IdleMode.kBrake;

    public static double wheelCircumference = 6;
    public static double gearratio = 0.1;

    public static double driveVelocityConversion = gearratio * wheelCircumference;
    
    public static double trackWidth = 0.1;
    

    public static final edu.wpi.first.math.geometry.Translation2d[] swerveModuleLocations = {
        new edu.wpi.first.math.geometry.Translation2d(trackWidth / 2.0, trackWidth / 2.0),
        new edu.wpi.first.math.geometry.Translation2d(trackWidth / 2.0, -trackWidth / 2.0),
        new edu.wpi.first.math.geometry.Translation2d(-trackWidth / 2.0, trackWidth / 2.0),
        new edu.wpi.first.math.geometry.Translation2d(-trackWidth / 2.0, -trackWidth / 2.0)
    };

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(swerveModuleLocations);

    }

}
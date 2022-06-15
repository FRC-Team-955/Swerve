package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class SwerveMod{
    public int moduleNumber;

    public double angleOffset;

    private CANSparkMax angleMotor;
    private CANCoder angleEncoder;
    private SparkMaxPIDController anglePID;

    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkMaxPIDController drivePID;

    private double lastAngle;

    private double anglekP;
    private double anglekI;
    private double anglekD;

    private double drivekP;
    private double drivekI;
    private double drivekD;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Settings.SwerveConstants.driveKS, Settings.SwerveConstants.driveKV, Settings.SwerveConstants.driveKA);

    public SwerveMod(int moduleNumber, int driveMotorID,  int angleMotorID,int cancoderID, double angleOffset){
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        // Angle Motor
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleMotor.setInverted(Settings.SwerveConstants.angleMotorInvert);
        angleMotor.setIdleMode(Settings.SwerveConstants.angleIdleMode);
        resetToAbsolute();

        // Angle Encoder
        angleEncoder = new CANCoder(cancoderID);
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(swerveCancoderConfig());

        // Angle PID
        anglePID = angleMotor.getPIDController();
        anglePID.setP(anglekP);
        anglePID.setI(anglekI);
        anglePID.setD(anglekD);

        // Drive Motor
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.setInverted(Settings.SwerveConstants.driveMotorInvert);
        driveMotor.setIdleMode(Settings.SwerveConstants.driveIdleMode);


        // Drive Encoder
        driveEncoder = driveMotor.getAlternateEncoder(8192);
        // driveEncoder = driveMotor.getEncoder();

        // driveEncoder.setVelocityConversionFactor(Settings.SwerveConstants.driveVelocityConversion);

        // Drive PID
        drivePID = driveMotor.getPIDController();
        drivePID.setP(drivekP);
        drivePID.setI(drivekI);
        drivePID.setD(drivekD);

        lastAngle = getState().angle.getDegrees();
    }

    public static CANCoderConfiguration swerveCancoderConfig() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = Settings.SwerveConstants.canCoderInvert;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        return config;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Settings.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        }else{
            double velocity = Conversions.MPSToNeo(desiredState.speedMetersPerSecond, Settings.SwerveConstants.driveGearRatio);
            drivePID.setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Settings.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees();
        anglePID.setReference(Conversions.degreesToNeo(angle, Settings.SwerveConstants.angleGearRatio), ControlType.kPosition);
        lastAngle = angle;
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToNeo(getCanCoder().getDegrees() - angleOffset, Settings.SwerveConstants.angleGearRatio);
        angleEncoder.setPosition(absolutePosition);
    }
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }


    public SwerveModuleState getState(){
        double velocity = Conversions.neoToMPS(driveEncoder.getVelocity(),Settings.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.neoToDegrees(angleEncoder.getAbsolutePosition(), Settings.SwerveConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
        // double velocity = driveMotor.getSelectedSensorVelocity() wheelCircumference, Constants.SwerveConstants.driveGearRatio
        // Rotation2d angle = Rotation2d.fromDegrees(Conversions.neoToDegrees(angleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio));
        // return new SwerveModuleState(velocity, angle);
    }

}
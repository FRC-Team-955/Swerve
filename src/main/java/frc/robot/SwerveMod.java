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
    private SparkMaxPIDController drivePID;

    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turningEncoder;     

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

        // angleEncoder = new CANCoder(turningCANCoderChannel);

        // Angle Motor
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleMotor.setInverted(Settings.SwerveConstants.angleMotorInvert);
        angleMotor.setIdleMode(Settings.SwerveConstants.angleIdleMode);

         // Drive Motor
         driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
         driveMotor.setInverted(Settings.SwerveConstants.driveMotorInvert);
         driveMotor.setIdleMode(Settings.SwerveConstants.driveIdleMode);   

        //TripleHelixCode
        angleEncoder = new CANCoder(cancoderID);
        // angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = Settings.SwerveConstants.canCoderInvert;
        // config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // config.sensorTimeBase = SensorTimeBase.PerSecond;
        angleEncoder.configAllSettings(config);     
        // angleEncoder.setPosition(0);    

        

        //Relative Encoders
        m_driveEncoder = driveMotor.getEncoder();
        m_turningEncoder = angleMotor.getEncoder();

        m_turningEncoder.setPositionConversionFactor(16.845);   //360.0 / 12.8

         // m_driveEncoder returns RPM by default. Use setVelocityConversionFactor() to
        // convert that to meters per second.
        //1.002187
        m_driveEncoder.setVelocityConversionFactor((1.0217 * 0.04284)/ 60.0);
        m_driveEncoder.setPositionConversionFactor(0.04284 * 1.0217); //(0.098 * Math.PI) / 6.75
        m_driveEncoder.setPosition(0);

        // Angle PID
        anglePID = angleMotor.getPIDController();
        anglePID.setP(Settings.SwerveConstants.angleKP);
        anglePID.setI(Settings.SwerveConstants.angleKI);
        anglePID.setD(Settings.SwerveConstants.angleKD);

        // Drive Encoder
        m_driveEncoder = driveMotor.getAlternateEncoder(42);
        m_driveEncoder = driveMotor.getEncoder();
        

        // driveEncoder.setVelocityConversionFactor(Settings.SwerveConstants.driveVelocityConversion);

        // Drive PID
        drivePID = driveMotor.getPIDController();
        drivePID.setP(drivekP);
        drivePID.setI(drivekI);
        drivePID.setD(drivekD);

        lastAngle = getState().angle.getDegrees();
        // resetToAbsolute();

    //a
    }

    // public static CANCoderConfiguration swerveCancoderConfig() {
        
    //     return config;
    // }

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
        System.out.print(moduleNumber + ": ");
        System.out.println(getCanCoder().getDegrees());
        
        // System.out.println(angle);
    }

      public double deltaAdjustedAngle(double targetAngle, double currentAngle) {

        return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    

    //TripleHelix 
    public void setDesiredState(SwerveModuleState state) {

        Rotation2d curAngle = Rotation2d.fromDegrees(m_turningEncoder.getPosition());

        double delta = deltaAdjustedAngle(state.angle.getDegrees(), curAngle.getDegrees());
        // return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;

        // Calculate the drive motor output from the drive PID controller.
        double driveOutput = state.speedMetersPerSecond;

        // if(this.moduleNumber == 0){
        //     driveOutput = -1;
        // }

        // if(this.moduleNumber == 0){
        //     SwerveDrive.turningdrive(swerveTranslation, swerveRotation, false, true);
        // }

        if (Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        adjustedAngle = Rotation2d.fromDegrees(delta + curAngle.getDegrees());

        anglePID.setReference(
            adjustedAngle.getDegrees(),
            ControlType.kPosition
        );        

        drivePID.setReference(driveOutput, ControlType.kVelocity, 0, 2.96 * driveOutput);
        // System.out.println(m_driveEncoder.getPosition());
        // .println(moduleNumber + "relative " + m_turningEncoder.getPosition());

    }
    
    public void setOpenLoopState(SwerveModuleState state) {
        Rotation2d curAngle = Rotation2d.fromDegrees(m_turningEncoder.getPosition());

        double delta = deltaAdjustedAngle(state.angle.getDegrees(), curAngle.getDegrees());

        // Calculate the drive motor output from the drive PID controller.
        double driveOutput = state.speedMetersPerSecond;

        if (Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        adjustedAngle = Rotation2d.fromDegrees(delta + curAngle.getDegrees());

        anglePID.setReference(
            adjustedAngle.getDegrees(),
            ControlType.kPosition
        );        

        // SmartDashboard.putNumber("Commanded Velocity", driveOutput);

        driveMotor.setVoltage(2.96 * driveOutput);
    }


    public void syncEncoders(){
        // m_turningEncoder.setPosition((angleEncoder.getAbsolutePosition() - angleOffset)*(374.599/189.668) - 45);
        // System.out.println(angleEncoder.getAbsolutePosition());
        m_turningEncoder.setPosition(angleEncoder.getAbsolutePosition()-angleOffset);
        
    }

    public void resetToAbsolute(){
        // Reset the cumulative rotation counts of the SparkMax motors
        m_turningEncoder.setPosition(0.0);

        angleEncoder.setPosition(0.0);
        // angleEncoder.configMagnetOffset(angleEncoder.configGetMagnetOffset() - angleEncoder.getAbsolutePosition());

        
    
        
    }
    public void resetDrive(){
        m_driveEncoder.setPosition(0.0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }


    public SwerveModuleState getState(){
        double m2 = -(m_turningEncoder.getPosition() % 360 + 360) % 360;

        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m2 * Math.PI / 180));
        // return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m2 * Math.PI / 180));
        // // double velocity = Conversions.neoToMPS(driveEncoder.getVelocity(),Settings.SwerveConstants.driveGearRatio);
        // double m2 = (angleEncoder.getPosition() % 360 + 360) % 360;
        // Rotation2d angle = Rotation2d.fromDegrees(Conversions.neoToDegrees(angleEncoder.getAbsolutePosition(), Settings.SwerveConstants.angleGearRatio));
        // return new SwerveModuleState(m2, angle);
        // // double velocity = driveMotor.getSelectedSensorVelocity() wheelCircumference, Constants.SwerveConstants.driveGearRatio
        // // Rotation2d angle = Rotation2d.fromDegrees(Conversions.neoToDegrees(angleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio));
        // // return new SwerveModuleState(velocity, angle);
    }

    public Rotation2d adjustedAngle = new Rotation2d();     

}
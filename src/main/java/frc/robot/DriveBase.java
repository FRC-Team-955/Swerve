package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import frc.robot.SwerveMod;

public class DriveBase {
    CANSparkMax frontLeft_drivemotor;
    CANSparkMax frontRight_drivemotor;
    CANSparkMax backLeft_drivemotor;
    CANSparkMax backRight_drivemotor;

    SparkMaxPIDController pidController_frontLeft_drivemotor;
    SparkMaxPIDController pidController_frontRight_drivemotor;
    SparkMaxPIDController pidController_backLeft_drivemotor;
    SparkMaxPIDController pidController_backRight_drivemotor;

    
    RelativeEncoder encoder_frontLeft_drivemotor;
    RelativeEncoder encoder_frontRight_drivemotor;
    RelativeEncoder encoder_backLeft_drivemotor;
    RelativeEncoder encoder_backRight_drivemotor;

    double kP = 0;
    double kI =0;
    double kD=0;
    double kFF=0;

    DriveBase(){
        init();
        initpid(pidController_frontLeft_drivemotor,kP,kI,kD,kFF);
        initpid(pidController_frontRight_drivemotor,kP,kI,kD,kFF);
        initpid(pidController_backLeft_drivemotor,kP,kI,kD,kFF);
        initpid(pidController_backRight_drivemotor,kP,kI,kD,kFF);
    }
    public void init(){
        frontLeft_drivemotor = new CANSparkMax(0, MotorType.kBrushless);
        frontRight_drivemotor = new CANSparkMax(1, MotorType.kBrushless);
        backLeft_drivemotor = new CANSparkMax(2, MotorType.kBrushless);
        backRight_drivemotor = new CANSparkMax(3, MotorType.kBrushless);
        
        pidController_frontLeft_drivemotor = frontLeft_drivemotor.getPIDController();
        pidController_frontRight_drivemotor = frontRight_drivemotor.getPIDController();
        pidController_backLeft_drivemotor = backLeft_drivemotor.getPIDController();
        pidController_backRight_drivemotor = backRight_drivemotor.getPIDController();

        encoder_frontLeft_drivemotor =  frontLeft_drivemotor.getAlternateEncoder(8192);
        encoder_frontRight_drivemotor =  frontRight_drivemotor.getAlternateEncoder(8192);
        encoder_backLeft_drivemotor =  backLeft_drivemotor.getAlternateEncoder(8192);
        encoder_backRight_drivemotor =  backRight_drivemotor.getAlternateEncoder(8192);
    }

    public void initpid(SparkMaxPIDController pidController, 
    double kP,double kI,double kD,double kFF){
        pidController.setP(0);
        pidController.setI(0);
        pidController.setD(0);
        pidController.setFF(0);
    }

    
}

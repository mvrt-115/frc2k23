package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonFactory;

import javax.xml.namespace.QName;

import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase{

    //private SparkMaxPIDController pidController;
    private TalonFX armMotor;
    private CANSparkMax clawMotor;
    private Logger logger;

    public GroundIntake(){
        armMotor = TalonFactory.createTalonFX(0, true);
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.setSelectedSensorPosition(degreesToTicks(Constants.GroundIntake.initialAngle));
        clawMotor = TalonFactory.createSparkMax(7, false);
        logger = Logger.getInstance();

     
        armMotor.config_kP(Constants.Talon.kPIDIdx, Constants.GroundIntake.kP);
        armMotor.config_kI(Constants.Talon.kPIDIdx, Constants.GroundIntake.kI);
        armMotor.config_kD(Constants.Talon.kPIDIdx, Constants.GroundIntake.kD);
        
        /* pidController = clawMotor.getPIDController();
        pidController.setP(Constants.Intake.kP);
        pidController.setI(Constants.Intake.kI);
        pidController.setD(Constants.Intake.kD);*/
        
    }

    public void periodic(){
        //This method will be called once per scheduler run
        //setPosition(30);
        log();
    }

    public void log(){
        logger.recordOutput("GroundIntake/armMotor/position_ticks", getArmCurrentPositionTicks());
        logger.recordOutput("GroundIntake/armMotor/position_degrees", getArmCurrentPositionDegrees());
        logger.recordOutput("GroundIntake/armMotor/percent_output", armMotor.getMotorOutputPercent());
        logger.recordOutput("GroundIntake/clawMotor/velocity", clawMotor.get());
    }

    public double getArmCurrentPositionTicks(){
        return armMotor.getSelectedSensorPosition();
    }
    
    public double getArmCurrentPositionDegrees(){
        return ticksToDegrees(getArmCurrentPositionTicks());
    }

    public double ticksToDegrees(double ticks){
        return ticks * (360.0/2048.0)/20.0;
    }

    public double degreesToTicks(double degrees){
        return degrees / ((360.0/2048.0)/20);
    }
    
    public void setPosition(double goalPositionDegrees){
        armMotor.set(
            ControlMode.Position, 
            degreesToTicks(goalPositionDegrees), 
            DemandType.ArbitraryFeedForward, 
            -Constants.GroundIntake.kG * Math.cos(Math.toRadians(getArmCurrentPositionDegrees()))/10
        );       
    }

    public void setClawSpeed(double speed){
        clawMotor.set(speed);
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonFactory;

import javax.xml.namespace.QName;

import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase{

    //private SparkMaxPIDController pidController;
    private TalonFX intakeMotor;
    private CANSparkMax armMotor;
    private Logger logger;
    private PIDController armPIDController;

    public GroundIntake(){
        armMotor = TalonFactory.createSparkMax(Constants.GroundIntake.kPivotPort, false );
        intakeMotor = TalonFactory.createTalonFX(Constants.GroundIntake.kIntakePort, false);
        logger = Logger.getInstance();

        armPIDController = new PIDController(Constants.GroundIntake.kP, Constants.GroundIntake.kI, Constants.GroundIntake.kD);

        armMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        armMotor.setIdleMode(IdleMode.kCoast);
        armMotor.getEncoder().setPosition(degreesToTicks(Constants.GroundIntake.initialAngle));
        
        
        
        /* pidController = clawMotor.getPIDController();
        pidController.setP(Constants.Intake.kP);
        pidController.setI(Constants.Intake.kI);
        pidController.setD(Constants.Intake.kD);*/
        
    }

    public void periodic(){
        //This method will be called once per scheduler run
        setPosition(30);
        log();
    }

    public void log(){
        logger.recordOutput("GroundIntake/armMotor/position_ticks", getArmCurrentPositionTicks());
        SmartDashboard.putNumber("position_ticks", getArmCurrentPositionTicks());
        logger.recordOutput("GroundIntake/armMotor/position_degrees", getArmCurrentPositionDegrees());
        SmartDashboard.putNumber("position_degrees", getArmCurrentPositionDegrees());
        logger.recordOutput("GroundIntake/armMotor/percent_output", armMotor.getAppliedOutput());
        logger.recordOutput("GroundIntake/clawMotor/velocity", intakeMotor.getSelectedSensorVelocity());
    }

    public double getArmCurrentPositionTicks(){
        return armMotor.getEncoder().getPosition();
    }
    
    public double getArmCurrentPositionDegrees(){
        return ticksToDegrees(getArmCurrentPositionTicks());
    }

    public double ticksToDegrees(double ticks){
        return ticks * 360.0/30.0;
    }

    public double degreesToTicks(double degrees){
        return degrees / ((360.0)/30.00);
    }
    
    public void setPosition(double goalPositionDegrees){

        double output = armPIDController.calculate(goalPositionDegrees - getArmCurrentPositionDegrees())/10.0;
        // output = Constants.GroundIntake.kG * Math.cos(Math.toRadians(getArmCurrentPositionDegrees()))/10.0;
        
        logger.recordOutput("GroundIntake/desiredOutput", output);

        
        armMotor.set(output);
    }

    public void setClawSpeed(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonFactory;;

/*
 * design: one neo to run motor
 */
public class Intake2 extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private DigitalInput prox;

  private SparkMaxPIDController pidController;
  
  public enum INTAKE_TYPE {wheeled, claw}; 
  private INTAKE_TYPE type;
  private Logger logger;

  /*
   * Initializes motor with encoder and proximity sensor, requiring the type of intake upon initialization
   * @param type    type of intake prototype
   */
  public Intake2(INTAKE_TYPE type) {
    this.type = type;

    motor = TalonFactory.createSparkMax(Constants.Intake.kMotorPort, false);
    encoder = motor.getEncoder(); 

    prox = new DigitalInput(Constants.Intake.kProximityPort); 

    pidController = motor.getPIDController();
    pidController.setP(Constants.Intake.kP);
    pidController.setI(Constants.Intake.kI);
    pidController.setD(Constants.Intake.kD);

    logger = Logger.getInstance();
  }

  /******************************************COMMANDS***************************************/

  public InstantCommand intake(){
    return new InstantCommand(() -> smoothRun(true));
  }

  public InstantCommand outtake(){
    return new InstantCommand(() -> smoothRun(false));
  }

  public InstantCommand runIn(){
    logger.recordOutput("Intake/isIntake", "hi");
    return new InstantCommand(() -> runMotor(true));
  }
  
  public InstantCommand runOut(){
    return new InstantCommand(() -> runMotor(false));
  }

  public InstantCommand stop(){
    return new InstantCommand(() -> stopIntaking());
  }

  /******************************************METHODS***************************************/

  public void stopIntaking()
  {
   /* if(prox.get())*/ motor.set(-Constants.Intake.kCompressedSpeed);
  }

  public void runMotor(boolean isIntaking)
  {
    double speed = isIntaking ? Constants.Intake.kGoalRPM : Constants.Intake.kOuttakeRPM;
   /* if(!isIntaking || isIntaking && !prox.get())*/ motor.set(speed);
  }

  public void smoothRun(boolean isIntaking)
  {
    double goalSpeed = isIntaking ? Constants.Intake.kGoalRPM : -Constants.Intake.kGoalRPM;
    /*if(!isIntaking || isIntaking && !prox.get())*/ pidController.setReference(goalSpeed, CANSparkMax.ControlType.kVelocity);
  }

  /******************************************PERIODIC***************************************/

   @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // stopIntaking();
    SmartDashboard.putBoolean("item in intake", !prox.get());
    logger.recordOutput("intake/current", motor.getOutputCurrent());
    logger.recordOutput("intake/element", !prox.get());
    logger.recordOutput("intake/voltage", motor.getAppliedOutput());
  }

  /******************************************CALCULATIONS***************************************/

  /**
   * @param speed
   * @return true if the speed of the motor is close to the desired compressing/extending speed
   */
  public boolean isWithinError(double speed)
  {
    return Math.abs(Math.abs(speed) - Constants.Intake.kGoalRPM) <= Constants.Intake.kMarginOfError;
  }

}
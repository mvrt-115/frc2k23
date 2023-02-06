// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonFactory;

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
  }

  /******************************************COMMANDS***************************************/

  public RunCommand intake(){
    return new RunCommand(() -> smoothRun(true));
  }

  public RunCommand outtake(){
    return new RunCommand(() -> smoothRun(false));
  }

  public RunCommand runIn(){
    return new RunCommand(() -> runMotor(true));
  }
  
  public RunCommand runOut(){
    return new RunCommand(() -> runMotor(false));
  }

  public RunCommand stop(){
    return new RunCommand(() -> motor.set(Constants.Intake.kCompressedSpeed));
  }

  /******************************************METHODS***************************************/

  public void runMotor(boolean isIntaking)
  {
    double speed = isIntaking ? Constants.Intake.kGoalRPM : -Constants.Intake.kGoalRPM;
    if(!isIntaking || isIntaking && !prox.get()) motor.set(speed);
  }

  public void smoothRun(boolean isIntaking)
  {
    double goalSpeed = isIntaking ? Constants.Intake.kGoalRPM : -Constants.Intake.kGoalRPM;
    if(!isIntaking || isIntaking && !prox.get()) pidController.setReference(goalSpeed, CANSparkMax.ControlType.kVelocity);
  }

  /******************************************PERIODIC***************************************/

   @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("item in intake", !prox.get());
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
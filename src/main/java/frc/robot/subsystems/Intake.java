// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonFactory;

/*
 * design: one neo to run motor
 */
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax motor;
  private DoubleSolenoid leftSolenoid, rightSolenoid; //only if using pistons
  private RelativeEncoder encoder;
  private DigitalInput proximityElement, 
                        proximityClaw; //proximity sensor used for telling when claw is 
  
  public enum INTAKE_TYPE {wheeled, claw}; 
  private INTAKE_TYPE type;

  /*
   * Initializes motor with encoder and proximity sensor, requiring the type of intake upon initialization
   * @param type    type of intake prototype
   */
  public Intake(INTAKE_TYPE type) {
    motor = TalonFactory.createSparkMax(0, false);
    encoder = motor.getEncoder();
    motor.setIdleMode(IdleMode.kBrake); 
    encoder.setPosition(Constants.Intake.kCompressedTicks); // initial position to when its preloaded
        //leftSolenoid = new DoubleSolenoid(0, null, 0, 0)

    proximityElement = new DigitalInput(Constants.Intake.kProximityPort); //CHANGE CHANNEL NUMBER FOR TESTING AND USE
    

    this.type = type;

    //
    if(this.type == INTAKE_TYPE.claw)
    {
      proximityClaw = new DigitalInput(Constants.Intake.kProximityClawPort); //CHANGE CHANNEL NUMBER 
    }
  }

  /////////////////////////////////////////COMMANDS//////////////////////////////////////////////

  public CommandBase intakeElement() {
    if(type == INTAKE_TYPE.wheeled)
      return intakeWheeled();
    else 
      return intakeClaw();
  }

  public CommandBase outtakeElement() {
    if(type == INTAKE_TYPE.wheeled)
      return outtakeWheeled();
    else 
      return outtakeClaw();
  }

  public CommandBase intakeClaw() {
    motor.setIdleMode(IdleMode.kCoast); // change to the correct method
    return this.runOnce(() -> motor.set(0.3));
   }

   public CommandBase outtakeClaw() {
    return new RunCommand(() -> this.expand());
   }

   public CommandBase intakeWheeled() { //needs to be worked on
    return null;
   }

   public CommandBase outtakeWheeled() { //needs to be worked on
    return null;
   }

   ////////////////////////////////////////METHODS////////////////////////////////////////////////

   /**
    * sets constant speed to claw until it reaches the limit of expansion as dictated by an encoder
    */
   public void expand() {
      motor.setIdleMode(IdleMode.kCoast); // change to the correct method
      motor.set(0);
      while(!(encoder.getPosition() <= Constants.Intake.kExpandedTicks + Constants.Intake.kMarginOfError && encoder.getPosition() >= Constants.Intake.kExpandedTicks - Constants.Intake.kMarginOfError ))
      {
        motor.set(-0.3);
      }
   }

  /*
   * Sets the motor to speed after entering coast mode
   * @param speed   speed to run motor at
   */
  public void setMotorSpeed(double speed) {
    motor.setIdleMode(IdleMode.kCoast);
    motor.set(speed);
  }

  /*
   * Sets the motor speed to zero and enter brake mode
   */
  public void zeroMotorSpeed() {
    setMotorSpeed(0);
    motor.setIdleMode(IdleMode.kBrake);
  }

   // unused method
   public double ticksToAngleInDegrees(double ticks){
      return (ticks/Constants.Intake.kTicksPerRotation)*360;
       // convert ticks to angle turned
   }

   @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("current ticks", encoder.getPosition());
  }

  

  /*
   * Planning for intake actions:
   * 
   *  consider which intake type: claw or wheeled
   * 
   * if claw:
   *  intake by running motor inwards and continue running to keep compression
   *  outtake by running motor outwards and continue until max 
   * 
   * if wheeled:
   *  intake by running wheels in after and setting to brake after proximity sensor detects element
   *  outtake by running wheels out for a set period of time
   */
  
}

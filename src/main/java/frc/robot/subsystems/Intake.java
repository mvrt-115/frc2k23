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
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private DigitalInput proximityElement; // infrared proximity sensor returns false when detecting object
  //private DigitalInput proximityClaw; //proximity sensor used for telling when claw is 

  private SparkMaxPIDController m_pidController;

  //proximityElement distance detected does not need to be tuned (tested)
  
  public enum INTAKE_TYPE {wheeled, claw}; 
  private INTAKE_TYPE type;

  /*
   * Initializes motor with encoder and proximity sensor, requiring the type of intake upon initialization
   * @param type    type of intake prototype
   */
  public Intake(INTAKE_TYPE type) {
    motor = TalonFactory.createSparkMax(0, false);
    encoder = motor.getEncoder(); // using encoder for claw prototype 
    motor.setIdleMode(IdleMode.kBrake); 
    encoder.setPosition(Constants.Intake.kCompressedTicks); // initial position of 0 to when its preloaded
    //initial state must always be preloaded

    proximityElement = new DigitalInput(0); //CHANGE CHANNEL NUMBER FOR TESTING AND USE
   // proximityClaw = new DigitalInput(1); // just for testing

    this.type = type;

    m_pidController = motor.getPIDController();
    m_pidController.setP(Constants.Intake.kP);
    m_pidController.setI(Constants.Intake.kI);
    m_pidController.setD(Constants.Intake.kD);
    m_pidController.setOutputRange(Constants.Intake.kMinOutput, Constants.Intake.kMaxOutput);

    // not using proximity claw anymore; using PID loop and breakmode during compression
    /*if(this.type == INTAKE_TYPE.claw)
    {
      proximityClaw = new DigitalInput(Constants.Intake.kProximityClawPort); //CHANGE CHANNEL NUMBER 
    }*/
  }

  /////////////////////////////////////////COMMANDS//////////////////////////////////////////////

  /*
   * Decides which intake method based on the type of intake system
   */
  public CommandBase intakeElement() {
    if(type == INTAKE_TYPE.wheeled)
      return intakeWheeled();
    else 
      return intakeClaw();
  }
  
  /*
   * Decides which outtake method based on the type of intake system
   */
  public CommandBase outtakeElement() {
    if(type == INTAKE_TYPE.wheeled)
      return outtakeWheeled();
    else 
      return outtakeClaw();
  }

  /*
   * Runs claw inwards constantly
   */
  public CommandBase intakeClaw() {
    motor.setIdleMode(IdleMode.kCoast); 
    return new RunCommand(() -> this.intakeAndHold());
        // change to running command with two steps: make sure its in brake mode after
   }

  /*
   * Runs claw outwards using expand()
   */
   public CommandBase outtakeClaw() {
    return new RunCommand(() -> this.moveClaw(false)); 
   }

   /*
    * Runs wheels inward and stops when the element is detected within the intake
    */
   public CommandBase intakeWheeled() {
    while(proximityElement.get()) {
      setMotorSpeed(0.3);
    }

    return new RunCommand(() -> zeroMotorSpeed());
   }

   /*
    * Runs wheels outward for a given period of time and then stops to outtake element
    */
   public CommandBase outtakeWheeled() {
    setMotorSpeed(-0.3);
    Timer.delay(1.5);
    return new RunCommand(() -> zeroMotorSpeed());
   }

   ////////////////////////////////////////METHODS////////////////////////////////////////////////

   /**
    * sets constant speed to claw until it reaches the limit of expansion as dictated by an encoder
    */
  /*  public void expand() {
      /*while(!(encoder.getPosition() <= Constants.Intake.kExpandedTicks + Constants.Intake.kMarginOfError && encoder.getPosition() >= Constants.Intake.kExpandedTicks - Constants.Intake.kMarginOfError))//(proximityClaw.get()) //if we have infrared proximity sensor in place 
      {
        setMotorSpeed(-0.3);
      }

      // pid the expansion
      motor.

      //zeroMotorSpeed();
   }
   */

   public void intakeAndHold()
   {
    this.moveClaw(true);
    motor.setIdleMode(IdleMode.kBrake);
   }
   /**
    * moves claw using PID control
    * prerequisite: the zero position should be the compressed position
    * @param isIntaking
    */
   public void moveClaw(boolean isIntaking)
   {
    //essentially, claw can only move between two set positions: contracted and holding item vs. expanded position
    double goalPos;
    if(isIntaking)
    {
      goalPos = Constants.Intake.kCompressedTicks; //change to desired ticks value when compressed
    }

    else
    {
      goalPos = Constants.Intake.kExpandedTicks; //change to desired ticks value when expanded
    }

    double rotations = goalPos/Constants.Intake.kTicksPerRotation;

    m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
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
    SmartDashboard.putBoolean("sensing element in intake", proximityElement.get());
   // SmartDashboard.putBoolean("sensing edge of elevator", proximityClaw.get());
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

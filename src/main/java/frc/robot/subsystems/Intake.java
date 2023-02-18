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
import edu.wpi.first.wpilibj2.command.Command;
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
  private DigitalInput proximityElement;

  private SparkMaxPIDController pidController;
  
  public enum INTAKE_TYPE {wheeled, claw}; 
  private INTAKE_TYPE type;

  public enum INTAKE_STATE {SPEEDING_UP, AT_SPEED, COMPRESSED};
  private INTAKE_STATE intakeState;

  private boolean prevProxState, currentProxState;

  /*
   * Initializes motor with encoder and proximity sensor, requiring the type of intake upon initialization
   * @param type    type of intake prototype
   */
  public Intake(INTAKE_TYPE type) {
    this.type = type;

    motor = TalonFactory.createSparkMax(0, false);
    encoder = motor.getEncoder(); 

    proximityElement = new DigitalInput(0); //CHANGE CHANNEL NUMBER FOR TESTING AND USE

    prevProxState = currentProxState = false; //false means that there is currently an item in the intake

    intakeState = INTAKE_STATE.COMPRESSED; 

    pidController = motor.getPIDController();
    pidController.setP(Constants.Intake.kP);
    pidController.setI(Constants.Intake.kI);
    pidController.setD(Constants.Intake.kD);
  }

  /******************************************COMMANDS***************************************/

  ///////////////////////GENERAL MANUAL//////////////////////////
 /*
   * Decides which intake method based on the type of intake system
   */
  public CommandBase manualIntake() {
    if(type == INTAKE_TYPE.wheeled)
      return manualIntakeWheeled();
    else 
      return manualIntakeClaw();
  }

  public CommandBase manualOuttake() {
    setMotorSpeed(-0.1);
    Timer.delay(1);
    return new RunCommand(() -> zeroMotorSpeed());
  }

  ///////////////////////WHEELED//////////////////////////

   public RunCommand outtakeElement()
   {
    while(intakeState != INTAKE_STATE.COMPRESSED)
    {
      moveClaw(false);
    }
    return new RunCommand(() -> setVelPIDSpeed(false, Constants.Intake.kGoalRPM));
   }

   public RunCommand intakeElement()
   {
    while(currentProxState) {} //wait until an object is detected in the vicinity

    while(intakeState != INTAKE_STATE.COMPRESSED) //after detected, run intake in set pattern
    {
      moveClaw(true);
    }
    return new RunCommand(() -> setVelPIDSpeed(true, Constants.Intake.kGoalRPM));
   }

   /*
    * Runs wheels inward and stops when the element is detected within the intake
    */
    public CommandBase manualIntakeWheeled() {
      setMotorSpeed(Constants.Intake.kGoalRPM);
      Timer.delay(2.5); ///ADJUST DELAY AS NECESSARY
  
      return new RunCommand(() -> setMotorSpeed(Constants.Intake.kCompressedSpeed)); //CHANGE HOLD SPEED AS NECESSARY
     }


     ///////////////////////CLAW//////////////////////////

  /*
   * Runs claw inwards constantly
   */
  public CommandBase manualIntakeClaw() {
    motor.setIdleMode(IdleMode.kCoast); // change to the correct method
    return this.runOnce(() -> motor.set(Constants.Intake.kGoalRPM));
   }

   /******************************************METHODS***************************************/

   ///////////////////////AUTO//////////////////////////

   /**
    * if it's intaking, then sets to the positive speed. if it is outtaking, then sets to the negative speed.
    * @param isIntaking
    */
   public void setVelPIDSpeed(boolean isIntaking, double desiredSpeed)
   {
      double speed = isIntaking ? desiredSpeed : -desiredSpeed;
      //double speed = isCompressing ? Constants.Intake.kGoalRPM : -Constants.Intake.kGoalRPM;
      pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
   }

   /**
    * this method is called when the operator constantly holds down the intaking button. 
    * speeds up the claw and claw closes
    */
   public void moveClaw(boolean isIntaking)
   {

    if(!isIntaking) intakeState = INTAKE_STATE.SPEEDING_UP;

    if(!currentProxState && isIntaking || !isIntaking)  // ADD SOFT LIMIT HERE USING ENCODER AND ALSO A CURRENT LIMIT
    {
        switch(intakeState)
        {
          case SPEEDING_UP:
          {
            setVelPIDSpeed(isIntaking, Constants.Intake.kGoalRPM);

            if(isWithinError(motor.get()))
            {
              intakeState = INTAKE_STATE.COMPRESSED;
            }
          }

          case AT_SPEED:
          {
           Timer.delay(1); // however much time it needs to intake the element
           intakeState = INTAKE_STATE.COMPRESSED;
          }

          case COMPRESSED:
          {
            if(isIntaking) setVelPIDSpeed(isIntaking, Constants.Intake.kGoalRPM);
            else zeroMotorSpeed();
          }
        }
    }

    /*//PROPOSED CHANGE: if intake button started intake and proximity sensor was used as a stopping mechanism to decide when to compress
    
    if(!currentProxState && isIntaking) 
    {
      while(!currentProxState) 
      {
        setVelPIDSpeed(isIntaking, Constants.Intake.kGoalRPM); //intaking at 0.3 until the proximity sensor is triggered
      }
      setVelPIDSpeed(isIntaking, Constants.Intake.kCompressedSpeed); //after in compressed position, switch to maintaining low compression speed
    }
    else if(!isIntaking) 
    {
      setVelPIDSpeed(isIntaking, Constants.Intake.kGoalRPM);
      Timer.delay(2); //CHANGE TIME BASED ON OUTTAKE TIME
      zeroMotorSpeed();
    }*/


   }

   ///////////////////////MANUAL//////////////////////////

   /**
    * sets constant speed to claw until it reaches the limit of expansion as dictated by an encoder
    */
   /*public void expand() {
      while(!(encoder.getPosition() <= Constants.Intake.kExpandedTicks + Constants.Intake.kMarginOfError && 
        encoder.getPosition() >= Constants.Intake.kExpandedTicks - Constants.Intake.kMarginOfError ))
      {
        setMotorSpeed(-Constants.Intake.kGoalRPM);
      }
      zeroMotorSpeed();
   }*/

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

  /******************************************PERIODIC***************************************/

   @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("sensing element in intake", proximityElement.get());
   // SmartDashboard.putBoolean("sensing edge of elevator", proximityClaw.get());

      currentProxState = SmartDashboard.getBoolean("prox state", true   );

      if(!currentProxState && prevProxState) //goes from no element to yes element
      {
        intakeState = INTAKE_STATE.SPEEDING_UP;
      }

      prevProxState = currentProxState;
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
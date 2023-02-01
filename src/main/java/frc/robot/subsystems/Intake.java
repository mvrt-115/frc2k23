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
    pidController.setOutputRange(Constants.Intake.kMinOutput, Constants.Intake.kMaxOutput);
  }

  /////////////////////////////////////////COMMANDS//////////////////////////////////////////////

   public RunCommand outtakeElement()
   {
     return new RunCommand(() -> moveClaw(false));
   }

   public RunCommand intakeElement()
   {
    return new RunCommand(() -> moveClaw(true));
   }

   /////////////////////////////////////////METHODS//////////////////////////////////////////////

   /**
    * if its compressing, then sets to the positive speed. if not, then sets to the negative speed.
    * @param isCompressing
    */
   public void setSpeed(boolean isCompressing)
   {
      double speed = isCompressing ? Constants.Intake.kGoalRPM : -Constants.Intake.kGoalRPM;
      pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
   }

   /**
    * this method is called when the operator constantly holds down the intaking button. 
    * speeds up the claw and claw closes
    */
   public void moveClaw(boolean isIntaking)
   {

    if(!isIntaking) intakeState = INTAKE_STATE.SPEEDING_UP;

    if(!currentProxState && isIntaking || !isIntaking) 
    {
        switch(intakeState)
        {
          case SPEEDING_UP:
          {
            setSpeed(isIntaking);

            if(isWithinError(motor.get()))
            {
              intakeState = INTAKE_STATE.AT_SPEED;
            }
          }

          case AT_SPEED:
          {
           Timer.delay(3); // however much time it needs to intake the element
           intakeState = INTAKE_STATE.COMPRESSED;
          }

          case COMPRESSED:
          {
            if(isIntaking) motor.set(0.03);
            else motor.set(0);
          }
        }
    }
   }

   @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("sensing element in intake", proximityElement.get());
   // SmartDashboard.putBoolean("sensing edge of elevator", proximityClaw.get());

      currentProxState = proximityElement.get();

      if(!currentProxState && prevProxState) //goes from no element to yes element
      {
        intakeState = INTAKE_STATE.SPEEDING_UP;
      }

      prevProxState = currentProxState;
  }

  /**
   * @param speed
   * @return true if the speed of the motor is close to the desired compressing/extending speed
   */
  public boolean isWithinError(double speed)
  {
    return Math.abs(Math.abs(speed) - Constants.Intake.kGoalRPM) <= Constants.Intake.kMarginOfError;
  }
  
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

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

  public Intake() {
    motor = TalonFactory.createSparkMax(0, false);
    encoder = motor.getEncoder();
    motor.setIdleMode(IdleMode.kBrake); 
    encoder.setPosition(Constants.Intake.kCompressedTicks); // initial position to when its preloaded
        //leftSolenoid = new DoubleSolenoid(0, null, 0, 0)
  }

   public CommandBase intakeElement() {
    motor.setIdleMode(IdleMode.kCoast); // change to the correct method
    return this.runOnce(() -> motor.set(0.3));
   }

   public CommandBase outtakeElement() {
    return new RunCommand(() -> this.expand());
   }

   /**
    * sets constant speed to claw until it reaches the limit of expansion
    */
   public void expand() {
      motor.setIdleMode(IdleMode.kCoast); // change to the correct methd
      motor.set(0);
      while(!(encoder.getPosition() <= Constants.Intake.kExpandedTicks + Constants.Intake.kMarginOfError && encoder.getPosition() >= Constants.Intake.kExpandedTicks - Constants.Intake.kMarginOfError ))
      {
        motor.set(-0.3);
      }
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

  /*public CommandBase intakeHold() {
    return this.runOnce(() -> leftMotor.set(0.3));
  }

  public CommandBase outtakeHold() {
    
  }

  public CommandBase intakePneumaticHold() {
    return this.runOnce(() -> )
  }

  public CommandBase outtakePneumaticHold() {
    
  }

  public CommandBase intakeWheels() {

  }

  public CommandBase outtakeWheels() {

  }*/

  
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * design: one neo to run motor
 */
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public CANSparkMax leftMotor, rightMotor;
  public DoubleSolenoid leftSolenoid, rightSolenoid;

  public Intake() {
    leftMotor = TalonFactory.createSparkMax(0, false);
    rightMotor = TalonFactory.createSparkMax(0, true);

    rightMotor.follow(leftMotor);

    leftSolenoid = new DoubleSolenoid()
  }

  public CommandBase intakeHold() {
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

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TalonFactory;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  public CANSparkMax leftMotor, rightMotor;

  public Drivetrain() {
    leftMotor = TalonFactory.createSparkMax(0, false);
    rightMotor = TalonFactory.createSparkMax(0, true);

    rightMotor.follow(leftMotor);
  }

  public CommandBase intakeHold() {
    return this.runOnce(() -> leftMotor.set(0.3));
  }

  public CommandBase outtakeHold() {
    return this.runOnce(() -> leftMotor.set(0.3));
  }

  public CommandBase intakePneumaticHold() {

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

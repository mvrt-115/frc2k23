// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeHPStation extends ParallelCommandGroup {
  /** Creates a new IntakeHPStation. */
  public IntakeHPStation(Elevator elevator, Intake2 intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetElevatorHeight(elevator, Constants.Elevator.INTAKE_HEIGHT), intake.runIn());
  }
}
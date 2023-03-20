// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreCone extends SequentialCommandGroup {
  /** Creates a new AutoScoreCone. */
  public AutoScoreCone(Elevator elevator, Intake2 intake ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intake.stop(),
      new SetElevatorHeight(elevator, Constants.Elevator.CONE_HIGH_HEIGHT, 0.25, 0.5),
      new BetterWaitCommand(0.2),
      intake.runOut(),
      new BetterWaitCommand(0.35),
      new ElevateDown(elevator),
      intake.stop()
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANdleLEDSystem;
import frc.robot.subsystems.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLevel extends SequentialCommandGroup {
  /** Creates a new AutoLevel. */
  public AutoLevel(SwerveDrivetrain drivetrain, double driveSpeed, CANdleLEDSystem cc) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForward(drivetrain, driveSpeed, 1.5),
      new Leveling(drivetrain, cc)
    );
  }
}

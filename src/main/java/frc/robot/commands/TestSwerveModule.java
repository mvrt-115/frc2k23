// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;

public class TestSwerveModule extends CommandBase {
  private final SwerveModule module;
  private final SwerveDrivetrain swerveDrivetrain;
  private final Timer timer;
  private final double period_s = 1.5; // seconds
  private int index;
  private double[] positions_deg = {0, 45, 90, 135, 180, 225, 270, 315, 360};
  /** Creates a new TestSwerveModule. */
  public TestSwerveModule(SwerveDrivetrain swerveDrivetrain, SwerveModule module) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.module = module;
    this.swerveDrivetrain = swerveDrivetrain;
    timer = new Timer();
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    index = -1;
    swerveDrivetrain.resetModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(index);
    index += timer.advanceIfElapsed(period_s)? 1:0;
    if (index >= 0 && index < positions_deg.length) {
      SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(positions_deg[index]));
      module.setRawState(state);
      // module.setAngle(Math.toRadians(positions_deg[index]));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    module.setRawState(new SwerveModuleState());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(index);
    return index >= positions_deg.length;
  }
}

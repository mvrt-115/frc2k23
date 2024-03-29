// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.MathUtils;

public class TestChassisSpeeds extends CommandBase {
  private final SwerveDrivetrain swerveDrivetrain;
  private final Timer timer;
  private final double period_s = 1.5; // seconds
  private final boolean testAll = false;
  private int index_x;
  private int index_y;
  private int index_w;
  private double[] x_speeds = {1, 0, -1, 0};
  private double[] y_speeds = {0, 1, 0, -1};
  private double[] w_speeds = {0, 0, 1, -1};
  
  private int index;
  private double[][] spesificTestCases = {
    {0, 0, 0}, // vx, vy, vw
    {1, 0, 0}, // Go forward
    {-1, 0, 0}, // Go backward

    {0, 1, 0}, // Go left
    {0, -1, 0}, // Go right

    {0, 0, 1}, // turn counter clockwise
    {0, 0, -1}, // turn counter counter clockwise

    {0, 1, 1}, // go left while turning counter clockwise
    {0, -1, 1}, // go right while turning counter clockwise
   
    {1, 0, 1}, // go forward while turning counter clockwise
    {-1, 0, 1}, // go backward while turning counter clockwise


    {1, 1, 0}, // diagonal towards forward and to the left
    {1, -1, 0}, // diagonal towards forward and to the right
    {-1, 1, 0}, // diagonal towards backward and to the left
    {-1, -1, 0}, // diagonal towards backward and to the right

    {1, 1, 1}, // diagonal towards forward and to the left while turning counter clockwise
    {1, -1, 1}, // diagonal towards forward and to the right while turning counter clockwise
    {-1, 1, 1}, // diagonal towards backward and to the left while turning counter clockwise
    {-1, -1, 1}, // diagonal towards backward and to the right while turning counter clockwise


  };
  /** Creates a new TestChassisSpeeds. */
  public TestChassisSpeeds(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrivetrain = drivetrain;
    timer = new Timer();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    index_x = 0;
    index_y = 0;
    index_w = 0;

    index = 0;
    swerveDrivetrain.resetModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(testAll) {
      if (timer.advanceIfElapsed(period_s)) {
        index_x += 1;
      }
      if (index_x >= x_speeds.length) {
        index_x = 0;
        index_y += 1;
      }
      if (index_y >= y_speeds.length) {
        index_y = 0;
        index_w += 1;
      }
      if (MathUtils.inRange(index_x, x_speeds.length) && MathUtils.inRange(index_y, y_speeds.length) && MathUtils.inRange(index_w, w_speeds.length)) {
        swerveDrivetrain.setSpeeds(x_speeds[index_x], y_speeds[index_y], w_speeds[index_w], Constants.SwerveDrivetrain.rotatePoints[0]);
      }
    } else {
      if (timer.advanceIfElapsed(period_s)) {
        index += 1;
      }
      if (MathUtils.inRange(index, spesificTestCases.length)) {
        swerveDrivetrain.setSpeeds(spesificTestCases[index][0], spesificTestCases[index][1], spesificTestCases[index][2], Constants.SwerveDrivetrain.rotatePoints[0]);
      }
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!(MathUtils.inRange(index_x, x_speeds.length) && MathUtils.inRange(index_y, y_speeds.length) && MathUtils.inRange(index_w, w_speeds.length)) && testAll) || (index >= spesificTestCases.length && !testAll);
  }
}

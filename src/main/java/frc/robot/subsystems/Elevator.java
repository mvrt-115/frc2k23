// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class Elevator extends SubsystemBase
{
  private ElevatorState currState;
  private SupplyCurrentLimitConfiguration currentConfig;
  private double lastError, lastTime;

  public enum ElevatorState {
    ZEROED, ZEROING, AT_SETPOINT, MOVING
  };
  
  private TalonFX elev_motor;
  
  /** Creates a new Elevator. */
  public Elevator()
  {
    currState = ElevatorState.ZEROED;
    
    elev_motor = new TalonFX(Constants.Elevator.MOTOR_ID);

    elev_motor.configFactoryDefault();

     // Sets up PIDF
     elev_motor.config_kP(Constants.Elevator.kPIDIdx, Constants.Elevator.P);
     elev_motor.config_kI(Constants.Elevator.kPIDIdx, Constants.Elevator.I);
     elev_motor.config_kD(Constants.Elevator.kPIDIdx, Constants.Elevator.D);
     elev_motor.config_kF(Constants.Elevator.kPIDIdx, Constants.Elevator.F);

    elev_motor.setNeutralMode(NeutralMode.Brake);

    elev_motor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Height", elev_motor.getSelectedSensorPosition()*Constants.Elevator.INCHES_PER_TICK);
    SmartDashboard.putNumber("Motor Velocity", elev_motor.getSelectedSensorVelocity()*Constants.Elevator.INCHES_PER_TICK);
    switch(currState) {
      case ZEROED:
      break;
      case AT_SETPOINT:
      break;
      case ZEROING:
      break;
      case MOVING:
      break;
    }
  }

  public void setHeightRaw(double targetHeightRaw)
  {
    ElevatorFeedforward feedForward = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV, Constants.Elevator.kA);
    double velocity = elev_motor.getSelectedSensorVelocity() * Constants.Elevator.INCHES_PER_TICK;
    feedForward.calculate(velocity);
    elev_motor.set(ControlMode.Position, targetHeightRaw, DemandType.ArbitraryFeedForward, Constants.Elevator.F);
  }

  public void setHeight(double targetHeight)
  {
    // Checks bounds
    targetHeight = targetHeight > Constants.Elevator.MAX_HEIGHT ? Constants.Elevator.MAX_HEIGHT:targetHeight;
    targetHeight = targetHeight < Constants.Elevator.MIN_HEIGHT ? Constants.Elevator.MIN_HEIGHT:targetHeight;
    
    setHeightRaw(targetHeight);
  }

  public double getHeight()
  {
    return elev_motor.getSelectedSensorPosition()*Constants.Elevator.INCHES_PER_TICK;
  }

  public void resetEncoder() {
    elev_motor.setSelectedSensorPosition(0);
  }

  public void setElevatorState(ElevatorState desiredState) {
    currState = desiredState;
  }

  public ElevatorState getElevatorState() {
    return currState;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class Elevator extends SubsystemBase
{
  private ElevatorState currState;
  // private SupplyCurrentLimitConfiguration currentConfig;

  public enum ElevatorState {
    ZEROED, ZEROING, AT_SETPOINT, MOVING
  };
  
  private TalonFX elev_motor;
  private PIDController pid;
  ElevatorFeedforward eFeedforward;
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State setpoint;

  /** Creates a new Elevator. */
  public Elevator()
  {
    currState = ElevatorState.ZEROED;

    eFeedforward = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV, Constants.Elevator.kA);
    
    elev_motor = new TalonFX(Constants.Elevator.MOTOR_ID);

    pid = new PIDController(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D);

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
    
    constraints = new TrapezoidProfile.Constraints(Constants.Elevator.MAX_VELOCITY, Constants.Elevator.MAX_ACCELERATION);
    goal = new TrapezoidProfile.State(targetHeightRaw, 0);
    setpoint = new TrapezoidProfile.State(elev_motor.getSelectedSensorPosition(), elev_motor.getSelectedSensorVelocity());
    TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
    setpoint = profile.calculate(0.02);
    double velocity = elev_motor.getSelectedSensorVelocity(); 
    double feedforward = eFeedforward.calculate(setpoint.velocity);
    elev_motor.set(ControlMode.Position, setpoint.position, DemandType.Neutral, (feedforward+pid.calculate(setpoint.velocity))/12);
    
    // elev_motor.set(ControlMode.PercentOutput, ((pid.calculate(getHeight(), targetHeightRaw)) + feedforward) / 10);
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

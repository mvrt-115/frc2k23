// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class Elevator extends SubsystemBase {
  private ElevatorState currState;
  private double targetHeight;
  private double currentHeight;
  // private SupplyCurrentLimitConfiguration currentConfig;

  public enum ElevatorState {
    ZEROED, ZEROING, AT_SETPOINT, GO_TO_SETPOINT
  };
  
  private TalonFX elev_motor;
  private PIDController pid;
  ElevatorFeedforward eFeedforward;
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State setpoint;
  private TrapezoidProfile profile;
  private DigitalInput inductiveSensor;

  /** Creates a new Elevator. */
  public Elevator(TalonFX elevatorMotor) {
    eFeedforward = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV, Constants.Elevator.kA);
    
    elev_motor = elevatorMotor;

    pid = new PIDController(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D);

    inductiveSensor = new DigitalInput(Constants.Elevator.SENSOR_PORT);

    constraints = new TrapezoidProfile.Constraints(Constants.Elevator.MAX_VELOCITY, Constants.Elevator.MAX_ACCELERATION);

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
    updateState();
    updateHeight();

    // SmartDashboard.putNumber("Elevator Level", getLevel());
    // SmartDashboard.putNumber("Elevator Target Heighr", targetHeight);
   // System.out.println("Elevator Target Height: " + targetHeight + " Level: " + getLevel());
    SmartDashboard.putNumber("Elevator Height", elev_motor.getSelectedSensorPosition()*Constants.Elevator.INCHES_PER_TICK);
    SmartDashboard.putNumber("Motor Velocity", elev_motor.getSelectedSensorVelocity()*Constants.Elevator.INCHES_PER_TICK);
  }
  
  public void keepAtHeight() {

  }

  public void updateHeight() {
    currentHeight = getHeight();
  }

  public void updateState() {
    if(isWithinError(targetHeight, 0) && !currentIsWithinError())
    {
      currState = ElevatorState.ZEROING;
    } 
    else if(isWithinError(targetHeight, 0) && currentIsWithinError()) {
      currState = ElevatorState.ZEROED;
      keepAtHeight();
    }
    else if(!currentIsWithinError()) {
      currState = ElevatorState.GO_TO_SETPOINT;
    }
    else {
      currState = ElevatorState.AT_SETPOINT;
      keepAtHeight();
    }
  }

  //to check that the height is in bounds
  public void setTargetHeight(double goal)
  {
    targetHeight = goal;
    // Checks bounds
    targetHeight = targetHeight > Constants.Elevator.MAX_HEIGHT ? Constants.Elevator.MAX_HEIGHT:targetHeight;
    targetHeight = targetHeight < Constants.Elevator.MIN_HEIGHT ? Constants.Elevator.MIN_HEIGHT:targetHeight;

    SmartDashboard.putNumber("Elevator target height", targetHeight);
    
    setHeightRaw(targetHeight);
  }
  
  
  private void setHeightRaw(double targetHeightRaw)
  {
    goal = new TrapezoidProfile.State(targetHeightRaw, 0);
    setpoint = new TrapezoidProfile.State(elev_motor.getSelectedSensorPosition(), elev_motor.getSelectedSensorVelocity());
    profile = new TrapezoidProfile(constraints, goal, setpoint);
    setpoint = profile.calculate(Constants.Elevator.KDt);
    
    double feedforward = eFeedforward.calculate(setpoint.velocity);
    // elev_motor.set(ControlMode.MotionMagic, setpoint.position, DemandType.ArbitraryFeedForward, (feedforward+pid.calculate(setpoint.velocity))/12);    
    
    elev_motor.set(ControlMode.PercentOutput, 1);
    currentHeight = getHeight();
    // double velocity = elev_motor.getSelectedSensorVelocity(); 
    // elev_motor.set(ControlMode.PercentOutput, ((pid.calculate(getHeight(), targetHeightRaw)) + feedforward) / 10);
  }

  

  public double getHeight()
  {
    return elev_motor.getSelectedSensorPosition()*Constants.Elevator.INCHES_PER_TICK;
  }

  public void resetEncoder() {
    if(isZeroed())
      elev_motor.setSelectedSensorPosition(0);
    else {
      setTargetHeight(Constants.Elevator.MIN_HEIGHT);
      elev_motor.setSelectedSensorPosition(0);
    }
  }

  public void setElevatorState(ElevatorState desiredState) {
    currState = desiredState;
  }

  public ElevatorState getElevatorState() {
    return currState;
  }

  public boolean isZeroed() {
    return inductiveSensor.get();
  }

  public int getLevel()
  {
    switch(currState)
    {
      case ZEROED:
        return 0;
      case ZEROING: 
        return 1;
      case GO_TO_SETPOINT:
        return 2;
      case AT_SETPOINT:
        return 3;
      default:
        return 1;
    }
  }

  public void setLevel(int level)
  {
    switch(level)
    {
      case 0:
        currState = ElevatorState.ZEROED;
      case 1:
        currState = ElevatorState.ZEROING;
      case 2:
        currState = ElevatorState.GO_TO_SETPOINT;
      case 3:
        currState = ElevatorState.AT_SETPOINT;
      default:
        currState = ElevatorState.ZEROING;
    }
  }

  public boolean currentIsWithinError() {
    return isWithinError(currentHeight, targetHeight);
  }

  private static boolean isWithinError(double current, double target)
  {
    return Math.abs(current-target) <= Constants.Elevator.ERROR;
  }

  // public void simulationPeriodic() {
  //   super.simulationPeriodic();

  //   updateHeight();
  //   updateState();
    
  //   SmartDashboard.putNumber("Elevator Level", getLevel());
  //   SmartDashboard.putNumber("Elevator Height", elev_motor.getSelectedSensorPosition()*Constants.Elevator.INCHES_PER_TICK);
  //   SmartDashboard.putNumber("Motor Velocity", elev_motor.getSelectedSensorVelocity()*Constants.Elevator.INCHES_PER_TICK);
  // }
}

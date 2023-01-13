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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class Elevator extends SubsystemBase
{
  private ElevatorState currState;
  private SupplyCurrentLimitConfiguration currentConfig;
  private final TrapezoidProfile.Constraints motor_vel_constraints = new TrapezoidProfile.Constraints(
    MathUtils.inchesToMeters(Constants.Elevator.MAX_VELOCITY),
    MathUtils.inchesToMeters(Constants.Elevator.MAX_ACCELERATION));

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private double custom_height;

  public enum ElevatorState {
    RESTING, GO_TO_MID, GO_TO_HIGH, GO_TO_SHELF, FREE
  };
  
  private TalonFX elev_motor;
  
  /** Creates a new Elevator. */
  public Elevator()
  {
    currState = ElevatorState.RESTING;
    
    elev_motor = new TalonFX(Constants.Elevator.MOTOR_ID);

    elev_motor.configFactoryDefault();

    /* DW abt PID rn
     // Sets up PIDF
     elev_motor.config_kP(Constants.Elevator.kPIDIdx, Constants.Elevator.P);
     elev_motor.config_kP(Constants.Elevator.kPIDIdx, Constants.Elevator.I);
     elev_motor.config_kP(Constants.Elevator.kPIDIdx, Constants.Elevator.D);
     elev_motor.config_kP(Constants.Elevator.kPIDIdx, Constants.Elevator.F);
    */

    elev_motor.setNeutralMode(NeutralMode.Brake);

    elev_motor.setSelectedSensorPosition(0);

    custom_height = 0;
  }

  @Override
  public void periodic()
  {
    runMotor();

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Height", elev_motor.getSelectedSensorPosition()*Constants.Elevator.INCHES_PER_TICK);
    SmartDashboard.putNumber("Motor Velocity", elev_motor.getSelectedSensorVelocity()*Constants.Elevator.INCHES_PER_TICK);
  }

  public double getHeightBasedOffState()
  {
    switch(currState)
    {
      // Later we check for what game object the intake is holding
      
      case RESTING:
        return Constants.Elevator.MIN_HEIGHT;
      case GO_TO_MID:
        return Constants.Elevator.CONE_MID_HEIGHT+Constants.Elevator.CONE_HEIGHT;
      case GO_TO_HIGH:
        return Constants.Elevator.CONE_HIGH_HEIGHT+Constants.Elevator.CONE_HEIGHT;
      case GO_TO_SHELF:
        return Constants.Elevator.SHELF_HEIGHT+Constants.Elevator.CONE_HEIGHT;
      case FREE:
        return custom_height;
      default:
        return Constants.Elevator.MIN_HEIGHT;
    }
  }

  public void runMotor()
  {
    var profile = new TrapezoidProfile(motor_vel_constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.calculate(Constants.Elevator.kDt);

    // Send setpoint to offboard controller PID
    elev_motor.set(
        ControlMode.MotionMagic,
        getHeightBasedOffState()/Constants.Elevator.INCHES_PER_TICK,
        DemandType.ArbitraryFeedForward,
        Constants.Elevator.kFF.calculate(m_setpoint.velocity) / 12.0);
  }

  public void setHeightRaw(double targetHeightRaw)
  {
    elev_motor.set(ControlMode.Position, targetHeightRaw);
  }

  public void setHeight(double targetHeight)
  {
    // Checks bounds
    targetHeight = targetHeight > Constants.Elevator.MAX_HEIGHT ? Constants.Elevator.MAX_HEIGHT:targetHeight;
    targetHeight = targetHeight < Constants.Elevator.MIN_HEIGHT ? Constants.Elevator.MIN_HEIGHT:targetHeight;
    
    setHeightRaw(targetHeight/Constants.Elevator.INCHES_PER_TICK);
  }

  public void setLevel(int level)
  {
    switch(level)
    {
      case 0:
        setResting();
      case 1:
        setMid();
      case 2:
        setHigh();
      case 3:
        setShelf();
      default:
        setResting();
    }
  }

  public int getLevel()
  {
    switch(currState)
    {
      case RESTING:
        return 0;
      case GO_TO_MID:
        return 1;
      case GO_TO_HIGH:
        return 2;
      case GO_TO_SHELF:
        return 3;
      default:
        return 0;
    }
  }

  public double getHeight()
  {
    return elev_motor.getSelectedSensorPosition()*Constants.Elevator.INCHES_PER_TICK;
  }

  public void setMid()
  {
    currState = ElevatorState.GO_TO_MID;
  }

  public void setHigh()
  {
    currState = ElevatorState.GO_TO_HIGH;
  }

  public void setShelf()
  {
    currState = ElevatorState.GO_TO_SHELF;
  }

  public void setFree()
  {
    currState = ElevatorState.FREE;
  }

  public void setResting()
  {
    currState = ElevatorState.RESTING;
  }

  public ElevatorState getState()
  {
    return currState;
  }

  public void setCustomHeight(double customHeight)
  {
    custom_height = customHeight;
  }

  public double getCustomHeight()
  {
    return custom_height;
  }
}

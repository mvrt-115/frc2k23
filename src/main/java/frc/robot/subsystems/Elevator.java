// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonFactory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.math.system.plant.DCMotor;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorState currState;
  private double targetHeight;
  private double currentHeight;
  private Logger logger;
  // private SupplyCurrentLimitConfiguration currentConfig;

  public enum ElevatorState {
    ZEROED, ZEROING, AT_SETPOINT, GO_TO_SETPOINT
  };
  
  private TalonFX elev_motor; // motor
  private TalonFX elev_motor2;
  private PIDController pid;  // controller
  ElevatorFeedforward eFeedforward; // elevator feed forward for motion magic
  private TrapezoidProfile.Constraints constraints; // max velocity and acceleration for motion magic
  private double startTime; // time that the motion magic starts 
  // trapezoid profile goal and current
   private TrapezoidProfile.State goal;
   private TrapezoidProfile.State initial;
   private TrapezoidProfile profile;
  private DigitalInput inductiveSensor; // inductive sensor to check that the elevators at the bottom

  // sim fields
  private Encoder encoder;
  private ElevatorSim sim;
  private EncoderSim encoderSim;
  public static DCMotor gearbox;
  public static TalonFXSimCollection elevMotorSim;


  /** Creates a new Elevator. */
  public Elevator() {
    eFeedforward = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV, Constants.Elevator.kA);
    
    elev_motor = TalonFactory.createTalonFX(Constants.Elevator.MOTOR_ID, false);
    elev_motor2 = TalonFactory.createTalonFX(Constants.Elevator.MOTOR_ID2, false);

    elev_motor2.follow(elev_motor);
    int forwardLimit = 23000;;
    int reverseLimit = -50;
    elev_motor.configForwardSoftLimitThreshold(forwardLimit);
    elev_motor.configReverseSoftLimitThreshold(reverseLimit);
    elev_motor.configForwardSoftLimitEnable(true, 0);
    elev_motor.configReverseSoftLimitEnable(true, 0);
    elev_motor2.configForwardSoftLimitThreshold(forwardLimit);
    elev_motor2.configReverseSoftLimitThreshold(reverseLimit);
    elev_motor2.configForwardSoftLimitEnable(true, 0);
    elev_motor2.configReverseSoftLimitEnable(true, 0);

    elev_motor.setNeutralMode(NeutralMode.Brake);
    elev_motor2.setNeutralMode(NeutralMode.Brake);

    
    pid = new PIDController(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D);

    inductiveSensor = new DigitalInput(Constants.Elevator.SENSOR_PORT);

    constraints = new TrapezoidProfile.Constraints(Constants.Elevator.MAX_VELOCITY, Constants.Elevator.MAX_ACCELERATION);

     // Sets up PIDF
     elev_motor.config_kP(Constants.Elevator.kPIDIdx, Constants.Elevator.P);
     elev_motor.config_kI(Constants.Elevator.kPIDIdx, Constants.Elevator.I);
     elev_motor.config_kD(Constants.Elevator.kPIDIdx, Constants.Elevator.D);
     elev_motor.config_kF(Constants.Elevator.kPIDIdx, Constants.Elevator.F);
     
    elev_motor.setSelectedSensorPosition(0);
    elev_motor2.setSelectedSensorPosition(0);

    logger = Logger.getInstance();

    // sim
    encoder = new Encoder(1, 2);
    encoder.reset();
    encoder.setDistancePerPulse(2.0 * Math.PI * Constants.Elevator.PULLEY_RADIUS / Constants.Elevator.GEAR_RATIO / 4096);
    encoderSim = new EncoderSim(encoder);
    elevMotorSim = elev_motor.getSimCollection();
    gearbox = DCMotor.getFalcon500(1);
    sim = new ElevatorSim(gearbox, Constants.Elevator.GEAR_RATIO, Constants.Elevator.MASS, Constants.Elevator.PULLEY_RADIUS, Constants.Elevator.BOTTOM, Constants.Elevator.TOP, true,
    VecBuilder.fill(0.01));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateState();
    updateHeight();
    log();
   // elev_motor.set
  }

  public void log() {
    // SmartDashboard.putNumber("Elevator Level", getLevel());
    // SmartDashboard.putNumber("Elevator Target Heighr", targetHeight);
   // System.out.println("Elevator Target Height: " + targetHeight + " Level: " + getLevel());
   SmartDashboard.putNumber("Elevator Height", elev_motor.getSelectedSensorPosition());
   // SmartDashboard.putNumber("elev 2 height", elev_motor2.getSelectedSensorPosition());
   // SmartDashboard.putNumber("Motor Velocity", elev_motor.getSelectedSensorVelocity());
   logger.recordOutput("Elevator/motor1/position_ticks", elev_motor.getSelectedSensorPosition());
//    logger.recordOutput("Elevator/motor2/position_ticks", elev_motor2.getSelectedSensorPosition());
   logger.recordOutput("Elevator/motor1/position_inches", ticksToInches(elev_motor.getSelectedSensorPosition()));
//   logger.recordOutput("Elevator/motor2/position_inches", ticksToInches(elev_motor2.getSelectedSensorPosition()));
   logger.recordOutput("Elevator/motor1/velocity", elev_motor.getSelectedSensorVelocity());
//    logger.recordOutput("Elevator/motor1/closed_loop_error", elev_motor.getClosedLoopError());
   logger.recordOutput("Elevator/motor1/percent_output", elev_motor.getMotorOutputPercent());
   logger.recordOutput("Elevator/motor1/output_current", elev_motor.getStatorCurrent());
   logger.recordOutput("Elevator/motor2/output_current", elev_motor.getStatorCurrent());
   logger.recordOutput("Elevator/motor2/percent_output", elev_motor2.getMotorOutputPercent());
  }

 
  public void keepAtHeight() {
    elev_motor.set(ControlMode.PercentOutput, (Constants.Elevator.kG)/10);
  }

  /* updates the height  */
  public void updateHeight() {
    currentHeight = getHeight();
  }

  /* updates the state */
  public void updateState() {
    if(isWithinError(targetHeight, 0) && !currentIsWithinError())
    {
      currState = ElevatorState.ZEROING;
    } 
    else if(isWithinError(targetHeight, 0) && currentIsWithinError()) {
      currState = ElevatorState.ZEROED;
     // keepAtHeight();
    }
    else if(!currentIsWithinError()) {
      currState = ElevatorState.GO_TO_SETPOINT;
    }
    else {
      currState = ElevatorState.AT_SETPOINT;
      //keepAtHeight();
    }
  }

  /** Checks that the height is in bounds and calls setHeightRaw
   * @param the goal height and the start time
  */
  public void setTargetHeight(double goalHeight, double startTime)
  {
    this.startTime = startTime;
    targetHeight = goalHeight;
    // Checks bounds
   // targetHeight = targetHeight > Constants.Elevator.MAX_HEIGHT ? Constants.Elevator.MAX_HEIGHT:targetHeight;
   // targetHeight = targetHeight < Constants.Elevator.MIN_HEIGHT ? Constants.Elevator.MIN_HEIGHT:targetHeight;

    goal = new TrapezoidProfile.State(ticksToInches(goalHeight), 0);
    initial = new TrapezoidProfile.State(ticksToInches(elev_motor.getSelectedSensorPosition()), ticksToInches(elev_motor.getSelectedSensorVelocity())*10);
    profile = new TrapezoidProfile(constraints, goal, initial);
  }
  
  /** uses motion magic to move the robot to the desired height
   * @param the goal height in ticks
   */ 
  public void setHeightRaw(double targetHeightRaw)
  {
    logger.recordOutput("Elevator/motor1/targetHeight", (targetHeightRaw));
    double t = Timer.getFPGATimestamp() - startTime; 
    TrapezoidProfile.State setpoint = profile.calculate(t);

    logger.recordOutput("Elevator/profile/setpointposition", setpoint.position);
    logger.recordOutput("Elevator/profile/setpointvelocity", setpoint.velocity);
    logger.recordOutput("Elevator/profile/goalvelocity", goal.velocity);
    logger.recordOutput("Elevator/profile/goalposition", goal.position);
    logger.recordOutput("Elevator/profile/initialvelocity", initial.velocity);
    logger.recordOutput("Elevator/profile/initialposition", initial.position);
    logger.recordOutput("Elevator/profile/t_relative", t);
    
     double feedforward = eFeedforward.calculate(setpoint.velocity);
    
     logger.recordOutput("Elevator/targetheight_in", ticksToInches(targetHeightRaw));
     logger.recordOutput("Elevator/targetheight_ticks", (targetHeightRaw));
     logger.recordOutput("Elevator/feedforward", feedforward);
     logger.recordOutput("Elevator/setvelocity", ((feedforward+pid.calculate(setpoint.velocity))/10));
     logger.recordOutput("Elevator/pidvalue", pid.calculate(setpoint.velocity));
     
    // sim
    //  elevMotorSim.setIntegratedSensorRawPosition((int)(setpoint.position));
    currentHeight = getHeight();

    elev_motor.set(ControlMode.Position, targetHeightRaw, DemandType.ArbitraryFeedForward, feedforward/10.0);

  }

  /** returns the height of the elevator
   * @return the height of the elevator in ticks
  */ 
  public double getHeight()
  {
    return elev_motor.getSelectedSensorPosition();
  }

  public double getVelocity() {
    return elev_motor.getSelectedSensorVelocity();
  }

  /* resets the encoder */
  public void resetEncoder() {
    if(isZeroed())
      elev_motor.setSelectedSensorPosition(0);
    else {
      setTargetHeight(Constants.Elevator.MIN_HEIGHT, Timer.getFPGATimestamp());
      elev_motor.setSelectedSensorPosition(0);
    }
  }
  
  // sets the elevator state
  public void setElevatorState(ElevatorState desiredState) {
    currState = desiredState;
  }

  /*  returns the elevator state
   *  @return the elevator state
    */
  public ElevatorState getElevatorState() {
    return currState;
  }

  /**  retuns true if the elevator is zeroed
   * @return whether or not the elevator is zeroed
  */
  public boolean isZeroed() {
    return inductiveSensor.get();
  }

  /* Returns the current level of the elevator in integer format
   * @return the current level of the elevator in integer format
   */
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

  /* sets the state of the elevator
   * @param the level to set the elevator in integer format
   */
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

  /* Returns true if the elevator is at the target height
   * @return whether or not the elevator is at the target height
   */
  public boolean currentIsWithinError() {
    return isWithinError(currentHeight, targetHeight);
  }

  /* returns if the elevator is at the right height with an error
   * 
   */
  private static boolean isWithinError(double current, double target)
  {
    return Math.abs(current-target) <= Constants.Elevator.ERROR;
  }

  public void runMotor(double speed) {
    elev_motor.set(ControlMode.PercentOutput, speed);
  }

  public double ticksToInches(double ticks) {
    return ticks/341.3;
  }

  public void stopMotors() {
    elev_motor.set(ControlMode.PercentOutput, 0);
  }
}

  /* public void simulationPeriodic() {
    super.simulationPeriodic();

    sim.update(0.020);

    sim.setInput(elev_motor.getMotorOutputVoltage());
    encoderSim.setDistance(sim.getPositionMeters());

    
    SmartDashboard.putNumber("Elevator Level", getLevel());
    SmartDashboard.putNumber("Elevator Height", elev_motor.getSelectedSensorPosition() * Constants.Elevator.INCHES_PER_TICK);
    //SmartDashboard.putNumber("Motor Velocity", elev_motor.getSelectedSensorVelocity());
  } */
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.math.system.plant.DCMotor;

public class Elevator extends SubsystemBase {
  private ElevatorState currState;
  private double targetHeight;
  private double currentHeight;
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
  // private TrapezoidProfile.State goal;
 // private TrapezoidProfile.State setpoint;
 // private TrapezoidProfile profile;
  private DigitalInput inductiveSensor; // inductive sensor to check that the elevators at the bottom

  // sim fields
  private Encoder encoder;
  private ElevatorSim sim;
  private EncoderSim encoderSim;
  public static DCMotor gearbox;
  public static TalonFXSimCollection elevMotorSim;


  /** Creates a new Elevator. */
  public Elevator(TalonFX elevatorMotor, TalonFX elevatorMotor2) {
    eFeedforward = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV, Constants.Elevator.kA);
    
    elev_motor = elevatorMotor;
    elev_motor2 = elevatorMotor2;
    elev_motor.setInverted(false);
    elev_motor2.setInverted(false);
    elev_motor2.follow(elev_motor);
    int forwardLimit = 10000;
    int reverseLimit = 50;
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

    elev_motor.configFactoryDefault();

     // Sets up PIDF
     elev_motor.config_kP(Constants.Elevator.kPIDIdx, Constants.Elevator.P);
     elev_motor.config_kI(Constants.Elevator.kPIDIdx, Constants.Elevator.I);
     elev_motor.config_kD(Constants.Elevator.kPIDIdx, Constants.Elevator.D);
     elev_motor.config_kF(Constants.Elevator.kPIDIdx, Constants.Elevator.F);

    elev_motor.setNeutralMode(NeutralMode.Brake);
    elev_motor.setSelectedSensorPosition(0);
    elev_motor2.setSelectedSensorPosition(0);

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
    // SmartDashboard.putNumber("Elevator Level", getLevel());
    // SmartDashboard.putNumber("Elevator Target Heighr", targetHeight);
   // System.out.println("Elevator Target Height: " + targetHeight + " Level: " + getLevel());
    SmartDashboard.putNumber("Elevator Height", elev_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("elev 2 height", elev_motor2.getSelectedSensorPosition());
    SmartDashboard.putNumber("Motor Velocity", elev_motor.getSelectedSensorVelocity());
   // elev_motor.set
  }
  
  public void keepAtHeight() {

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
   // System.out.println("hi");
    targetHeight = goalHeight;
    // Checks bounds
   // targetHeight = targetHeight > Constants.Elevator.MAX_HEIGHT ? Constants.Elevator.MAX_HEIGHT:targetHeight;
   // targetHeight = targetHeight < Constants.Elevator.MIN_HEIGHT ? Constants.Elevator.MIN_HEIGHT:targetHeight;

   // SmartDashboard.putNumber("Elevator target height", targetHeight);
 //  goal = new TrapezoidProfile.State(targetHeight, 0);
  // setpoint = new TrapezoidProfile.State();
   //profile = new TrapezoidProfile(constraints, goal, setpoint);
    setHeightRaw(targetHeight);
  }
  
  /** uses motion magic to move the robot to the desired height
   * @param the goal height
   */ 
  private void setHeightRaw(double targetHeightRaw)
  {
    System.out.println("hi");
    TrapezoidProfile.State goal = new TrapezoidProfile.State(targetHeightRaw, 0);
    TrapezoidProfile.State setpoint = new TrapezoidProfile.State(elev_motor.getSelectedSensorPosition(), elev_motor.getSelectedSensorVelocity());
    TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
    SmartDashboard.putNumber("target height", targetHeightRaw);
    SmartDashboard.putNumber("goal position", goal.position);
    //SmartDashboard.putNumber("profile info", profile);
    // SmartDashboard.putString("setpoint", setpoint.to) 
    setpoint = profile.calculate(Timer.getFPGATimestamp() - startTime);
    SmartDashboard.putNumber("setpoint position", setpoint.position);
    
    double feedforward = eFeedforward.calculate(setpoint.velocity);
    SmartDashboard.putNumber("feedforward", feedforward);
    //pid.setSetpoint(setpoint.position);
    SmartDashboard.putNumber("pid", pid.calculate(setpoint.velocity));
    SmartDashboard.putNumber("setpoint velocity", setpoint.velocity);
    SmartDashboard.putNumber("velocity", (feedforward+pid.calculate(setpoint.velocity))/12);
    elev_motor.set(ControlMode.MotionMagic, setpoint.position, DemandType.ArbitraryFeedForward, (feedforward+pid.calculate(setpoint.velocity))/12);
    //SmartDashboard.putNumber("Elevator Height", elev_motor.getSelectedSensorPosition());
    // sim
    //  elevMotorSim.setIntegratedSensorRawPosition((int)(setpoint.position));
    currentHeight = getHeight();

    // double velocity = elev_motor.getSelectedSensorVelocity(); 
    //elev_motor.set(ControlMode.PercentOutput, ((pid.calculate(getHeight(), targetHeightRaw)) + feedforward) / 10);
  }

  /** returns the height of the elevator
   * @return the height of the elevator in ticks
  */ 
  public double getHeight()
  {
    return elev_motor.getSelectedSensorPosition();
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
    SmartDashboard.putNumber("Elevator Height", elev_motor.getSelectedSensorPosition());
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
}

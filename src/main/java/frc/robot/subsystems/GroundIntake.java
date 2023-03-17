package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonFactory;

import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase{

    private TalonFX rollerMotor;
    private CANSparkMax armMotor;
    private Logger logger;
    private PIDController armPIDController;

    public GroundIntake(){
        armMotor = TalonFactory.createSparkMax(Constants.GroundIntake.kPivotId, false );
        rollerMotor = TalonFactory.createTalonFX(Constants.GroundIntake.kRollerId, false);
        logger = Logger.getInstance();

        armPIDController = new PIDController(Constants.GroundIntake.kP, Constants.GroundIntake.kI, Constants.GroundIntake.kD);

        armMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.getEncoder().setPosition(degreesToRotations(Constants.GroundIntake.initialAngle));
    }

    public void periodic(){
        log();

        
    }

    public void log(){
        // logger.recordOutput("GroundIntake/armMotor/position_ticks", getArmCurrentPositionTicks());
        // logger.recordOutput("GroundIntake/armMotor/position_degrees", getArmCurrentPositionDegrees());
        // SmartDashboard.putNumber("gi - arm pos deg", getArmCurrentPositionDegrees());
        // SmartDashboard.putNumber("gi - arm output", armMotor.getAppliedOutput());
        // logger.recordOutput("GroundIntake/armMotor/percent_output", armMotor.getAppliedOutput());
        // logger.recordOutput("GroundIntake/clawMotor/velocity", rollerMotor.getSelectedSensorVelocity());
    }

    public void resetPID() {
        armPIDController.reset();
    }

    public void setRollerOutput(double output) {
        rollerMotor.set(ControlMode.PercentOutput, output);
    }

    public void stopRoller() {
        setRollerOutput(0.05);
    }

    public double getArmCurrentPositionTicks(){
        return armMotor.getEncoder().getPosition();
    }
    
    public double getArmCurrentPositionDegrees(){
        return rotationsToDeegrees(getArmCurrentPositionTicks());
    }

    public double rotationsToDeegrees(double degrees){
        return degrees * 360.0/Constants.GroundIntake.kGearRatio;
    }

    public double degreesToRotations(double degrees){
        return degrees / ((360.0)/Constants.GroundIntake.kGearRatio);
    }

    public double getFeedForward() {
        return Constants.GroundIntake.kG * Math.cos(Math.toRadians(getArmCurrentPositionDegrees()))/10.0;
    }
    
    public void setPosition(double goalPositionDegrees){

        double pidOutput = armPIDController.calculate(getArmCurrentPositionDegrees() - goalPositionDegrees)/10.0;
        
        double output = pidOutput + getFeedForward();

        logger.recordOutput("GroundIntake/desiredOutput", output);

        
        if(output > 0.7)
            output = 0.7;

        setArmOutput(output);
    }

    public void setArmOutput(double output) {
        armMotor.set(output);
    }
}
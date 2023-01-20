package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.*;

public class ClawIntake extends Intake {
    private final DigitalInput proximityClaw; //proximity sensor used for telling when claw is

    public ClawIntake() {
        super(Type.CLAW);

        proximityClaw = new DigitalInput(Constants.Intake.kProximityClawPort); //CHANGE CHANNEL NUMBER
    }

    public void close() {
        proximityClaw.close();
    }

    public CommandBase intakeElement() {
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast); // change to the correct method
        return this.runOnce(() -> motor.set(0.3));
    }

    public CommandBase outtakeElement() {
        return new RunCommand(() -> this.expand());
    }


    /**
     * sets constant speed to claw until it reaches the limit of expansion as dictated by an encoder
     */
    public void expand() {
        while (!proximityClaw.get()) //if we have proximity sensor in place
        //!(encoder.getPosition() <= Constants.Intake.kExpandedTicks + Constants.Intake.kMarginOfError && encoder.getPosition() >= Constants.Intake.kExpandedTicks - Constants.Intake.kMarginOfError ))
        {
            setMotorSpeed(-0.3);
        }
        zeroMotorSpeed();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("current ticks", encoder.getPosition());
    }
}

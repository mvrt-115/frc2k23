package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;

public class WheelIntake extends Intake {
    private final DigitalInput proximityElement;

    public WheelIntake() {
        super(Type.WHEEL);

        proximityElement = new DigitalInput(Constants.Intake.kProximityPort); //CHANGE CHANNEL NUMBER FOR TESTING AND USE
    }

    public void close() {
        proximityElement.close();
    }

    public edu.wpi.first.wpilibj2.command.CommandBase intakeElement() {
        while (!proximityElement.get()) {
            setMotorSpeed(0.3);
        }

        return new RunCommand(() -> zeroMotorSpeed());
    }

    public edu.wpi.first.wpilibj2.command.CommandBase outtakeElement() {
        setMotorSpeed(-0.3);
        edu.wpi.first.wpilibj.Timer.delay(1.5);
        return new RunCommand(() -> zeroMotorSpeed());
    }
}

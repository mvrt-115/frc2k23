package frc.robot.subsystems;

public class IntakeFactory {
    public static Intake New(Intake.Type type) {
        return type == Intake.Type.WHEEL ?
                new WheelIntake() :
                new ClawIntake();
    };
}

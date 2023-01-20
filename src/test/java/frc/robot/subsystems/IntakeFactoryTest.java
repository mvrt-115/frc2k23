package frc.robot.subsystems;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

class IntakeFactoryTest {
    Intake intake;

    @AfterEach
    public void closeIntake() {
        intake.close();
    }

    @Test
    public void newShouldReturnWheelIntake() {
        intake = IntakeFactory.New(Intake.Type.WHEEL);

        Assertions.assertInstanceOf(WheelIntake.class, intake);
    }

    @Test
    public void newShouldReturnClawIntake() {
        intake = IntakeFactory.New(Intake.Type.CLAW);

        Assertions.assertInstanceOf(ClawIntake.class, intake);
    }
}
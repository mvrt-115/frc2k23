package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.DemandType;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import static org.mockito.Mockito.doNothing;

import frc.robot.Constants;

@ExtendWith(MockitoExtension.class)
public class ElevatorTest {
    Elevator elevator;

    @Test 
    public void testSetHeightWithinRange(@Mock TalonFX elevatorMotor) {
        doNothing().when(elevatorMotor).set(ControlMode.MotionMagic, 10, DemandType.ArbitraryFeedForward, 0);
        elevator = new Elevator(elevatorMotor);
        elevator.setTargetHeight((Constants.Elevator.MIN_HEIGHT + Constants.Elevator.MAX_HEIGHT) / 2);
    }
}

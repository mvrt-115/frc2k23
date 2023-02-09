package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class MathUtilsTest {
    @Test
    public void testHandleDeadband() {
        double delta = Double.MIN_VALUE;

        assertEquals(1.0/3.0, MathUtils.handleDeadband(.5, .25), delta, "");
    }
}

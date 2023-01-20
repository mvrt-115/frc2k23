// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonFactory;

/*
 * design: one neo to run motor
 */
public abstract class Intake extends SubsystemBase {
    public enum Type {
        WHEEL,
        CLAW,
    }

    protected final CANSparkMax motor;
    protected final RelativeEncoder encoder;

    public Intake(Type id) {
        motor = TalonFactory.createSparkMax(id.ordinal(), false);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        encoder = motor.getEncoder();
        encoder.setPosition(Constants.Intake.kCompressedTicks); // initial position to when its preloaded
    }

    public abstract void close();

    /*
     * Decides which intake method based on the type of intake system
     */
    public abstract CommandBase intakeElement();

    /*
     * Decides which outtake method based on the type of intake system
     */
    public abstract CommandBase outtakeElement();

    // unused method
    public static double ticksToAngleInDegrees(double ticks) {
        return (ticks / Constants.Intake.kTicksPerRotation) * 360;
        // convert ticks to angle turned
    }

    /*
     * Sets the motor to speed after entering coast mode
     * @param speed   speed to run motor at
     */
    public void setMotorSpeed(double speed) {
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.set(speed);
    }

    /*
     * Sets the motor speed to zero and enter brake mode
     */
    public void zeroMotorSpeed() {
        setMotorSpeed(0);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /*
     * Planning for intake actions:
     *
     *  consider which intake type: claw or wheeled
     *
     * if claw:
     *  intake by running motor inwards and continue running to keep compression
     *  outtake by running motor outwards and continue until max
     *
     * if wheeled:
     *  intake by running wheels in after and setting to brake after proximity sensor detects element
     *  outtake by running wheels out for a set period of time
     */

}

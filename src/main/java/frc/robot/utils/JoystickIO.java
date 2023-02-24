package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * To add functionality to the CommandJoystick without having to refactor too much code, we 
 * made a joystickIO class that stores joystick inversions, swaps, POV, and other utility
 */
public class JoystickIO extends CommandJoystick{
    private boolean xBoxControl = false;
    private boolean leftStickInverted = false;
    private boolean rightStickInverted = false;
    private double invX = 1, invY = 1, invW = 1, invZ = 1;

    public JoystickIO(int port) {
        super(port);
    }

    public JoystickIO(int port, boolean xBoxControl, boolean invertLeftStick, boolean invertRightStick) {
        super(port);
        this.xBoxControl = xBoxControl;
        this.leftStickInverted = invertLeftStick;
        this.rightStickInverted = invertRightStick;
    }

    public JoystickIO(int port, boolean invertLeftStick, boolean invertRightStick) {
        super(port);
        this.leftStickInverted = invertLeftStick;
        this.rightStickInverted = invertRightStick;
        this.xBoxControl = false;
    }

    /**
     * switch X & Y axis on left stick
     */
    public void invertLeftStick() {
        leftStickInverted = true;
    }

    /**
     * reset X & Y axis on left stick
     */
    public void resetLeftStick() {
        leftStickInverted = false;
    }

    /**
     * switch W & Z axis on right stick
     */
    public void invertRightStick() {
        rightStickInverted = true;
    }

    /**
     * reset W & Z axis on right stick
     */
    public void resetRightStick() {
        rightStickInverted = false;
    }

    /**
     * invert X axis (left was - now +, right was + now -)
     */
    public void invertJoystickX() {
        invX = -1;
    }

    /**
     * invert Y axis (down was - now +, up was + now -)
     */
    public void invertJoystickY() {
        invY = -1;
    }

    /**
     * invert W axis (left was - now +, right was + now -)
     */
    public void invertJoystickW() {
        invW = -1;
    }

    /**
     * inver Z axis (down was - now +, up was + now -)
     */
    public void invertJoystickZ() {
        invZ = -1;
    }

    /**
     * This class always expects axis to be 0,1,4,5
     * <ul>
     * <li>0 = X</li>
     * <li>1 = Y</li>
     * <li>4 = W</li>
     * <li>5 = Z</li>
     * </ul>
     * Unless axis are inverted:
     * <ul>
     *  <li>left on the left stick means left (- X values),</li>
     *  <li>up on the left stick means up (+ Y values),</li>
     *  <li>left on the right stick means left (+ W values),</li>
     *  <li>up on the right stick means up (+ Z values)</li>
     * </ul>
     * All other axis are handled by the Parent Class
     */
    public double getRawAxis(int axis) {
        if (xBoxControl) {
            // xBoxAxis are inverted so up is X and left is Y etc. 
            switch (axis) {
                case 0: return invX * (leftStickInverted ? super.getRawAxis(0) : super.getRawAxis(1));
                case 1: return invY * (leftStickInverted ? super.getRawAxis(1) : super.getRawAxis(0));
                case 4: return invW * (rightStickInverted ? super.getRawAxis(4) : super.getRawAxis(5));
                case 5: return invZ * (rightStickInverted ? super.getRawAxis(5) : super.getRawAxis(4));
            }
        }
        else {
            switch (axis) {
                case 0: return invX * (leftStickInverted ? super.getRawAxis(1) : super.getRawAxis(0));
                case 1: return invY * (leftStickInverted ? super.getRawAxis(0) : super.getRawAxis(1));
                case 4: return invW * (rightStickInverted ? super.getRawAxis(5) : super.getRawAxis(4));
                case 5: return invZ * (rightStickInverted ? super.getRawAxis(4) : super.getRawAxis(5));
            }
        }
        return super.getRawAxis(axis);
    }
}
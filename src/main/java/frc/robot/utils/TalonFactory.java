package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class TalonFactory {
    /**
     * Creates a basic TalonFX with basic configurations
     * 
     * @param id        the id of the Falcon on the robot (get from PheonixTuner)
     * @param inversion the inversion of the TalonFX (false to spin forward, true to spin backwards)
     * @return          the generated TalonFX object
     */
    public static TalonFX createTalonFX(int id, boolean inversion) {
        TalonFX talon = new TalonFX(id);
        talon.configFactoryDefault();
        talon.configSupplyCurrentLimit(Constants.Talon.kCurrentLimit, Constants.Talon.kTimeoutMs);
        talon.configOpenloopRamp(0.4, Constants.Talon.kTimeoutMs);
        talon.setInverted(inversion);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.Talon.kPIDIdx, Constants.Talon.kTimeoutMs);
        talon.configVoltageCompSaturation(Constants.Talon.kVoltageComp, Constants.Talon.kTimeoutMs);
        talon.enableVoltageCompensation(true);       
        
        return talon;
    }
    
    /**
     *  Creates a basic TalonSRX with basic configurations
     * 
     *  @param id        the id of the Falcon on the robot (get from PheonixTuner)
     *  @param inversion the inversion of the TalonSRX (false to spin forward, true to spin backwards)
     *  @return          the generated TalonSRX object
     */
    public static TalonSRX createTalonSRX(int id,  boolean inversion) {
        TalonSRX talon = new TalonSRX(id);
        talon.configFactoryDefault();
        talon.configSupplyCurrentLimit(Constants.Talon.kCurrentLimit, Constants.Talon.kTimeoutMs);
        talon.configOpenloopRamp(0.4, Constants.Talon.kTimeoutMs);
        talon.setInverted(inversion);
        talon.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, Constants.Talon.kPIDIdx, Constants.Talon.kTimeoutMs);
        talon.configVoltageCompSaturation(Constants.Talon.kVoltageComp, Constants.Talon.kTimeoutMs);
        talon.enableVoltageCompensation(true);       
        
        return talon;
    }
}
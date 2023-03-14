// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleLEDSystem extends SubsystemBase {
  private final CANdle candle = new CANdle(0);
  private final StrobeAnimation yell = new StrobeAnimation(150, 150, 20, 0, 1, 300);
  private final StrobeAnimation purp = new StrobeAnimation(200, 0, 200, 20, 1, 300);
  RainbowAnimation whileLeveling = new RainbowAnimation(.8, 0.8, 300);

  /** true for cube, false for cone */
  boolean prevCubeCone = false;
  boolean cubeCone = false;
  boolean leveling = false;

 
  /** Creates a new CANdleLEDSystem. */
  public CANdleLEDSystem() {
   candle.configFactoryDefault();
    candle.configBrightnessScalar(.2);
  }

  
  public void setLeveling(boolean aboutToLevel) {
    leveling = aboutToLevel;
  }
  public void toggleCubeCone(){
    cubeCone = !cubeCone;
  }
  
 
  private void setPrupMethod() {
    // setColor(200, 0, 200);
    candle.animate(purp);
  }
  private void setYellowMethod() {
  //  cand
    // setColor(0, 255, 0)
    candle.animate(yell);
  }
  private void updateLEDCubeCone() {
    // System.out.println("UPADTEDDDddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddD");
    if (cubeCone) {
      setPrupMethod();
    }
    else {
      setYellowMethod();
    }
  }

  

  @Override
  public void periodic() {
    //you have to compare prev network tables state to new one to see if it changed because 
    //CANdle cannot be set periodically or else it will lag
    // if(prevCubeCone != cubeCone){
    //   updateLEDCubeCone();
    //   // System.out.println("UPDATED TO: " + cubeCone);
    // }
    // prevCubeCone = cubeCone;
    // candle.get
    if (leveling) {
      candle.animate(whileLeveling);
    }
    else{

      if (cubeCone) {
        setPrupMethod();
      }
      else {
        setYellowMethod();
      }
    }
    // RainbowAnimation r = new RainbowAnimation(1, 0.55, 100);
    // candle.animate(r);
  }
}

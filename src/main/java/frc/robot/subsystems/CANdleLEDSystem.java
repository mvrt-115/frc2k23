// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleLEDSystem extends SubsystemBase {
  private final CANdle candle = new CANdle(0);
  private final StrobeAnimation yell = new StrobeAnimation(200, 150, 0, 0, 1, 300);
  private final StrobeAnimation purp = new StrobeAnimation(255, 0, 255, 20, 1, 300);
  private final StrobeAnimation green = new StrobeAnimation(0, 200, 200, 20, 0.5, 300);
  private final StrobeAnimation greenFin = new StrobeAnimation(0, 200, 200, 20, 1, 300);
  RainbowAnimation whileLeveling = new RainbowAnimation(.8, 0.8, 300);

  /** true for cube, false for cone */
  boolean prevCubeCone = false;
  boolean cubeCone = false;
  boolean leveling = false;

  boolean aligning = false;
  boolean finisehdAligning = false;
  double alignError = 0;
  Timer greenFlash;
  double greenFlashTime = 3;

  /** Creates a new CANdleLEDSystem. */
  public CANdleLEDSystem() {
    candle.configFactoryDefault();
    candle.configBrightnessScalar(.2);
    greenFlash = new Timer();
  }

  public void setLeveling(boolean aboutToLevel) {
    leveling = aboutToLevel;
  }

  public void setAligning(boolean nextAlign) {
    aligning = nextAlign;
  }
  public void setFinAligning(boolean nextFinAlign) {
    finisehdAligning = nextFinAlign;
  }

  public void toggleCubeCone() {
    cubeCone = !cubeCone;
  }

  private void setPrupMethod() {
    // setColor(200, 0, 200);
    candle.animate(purp);
  }

  private void setYellowMethod() {
    // cand
    // setColor(0, 255, 0)
    candle.animate(yell);
  }

  private void updateLEDCubeCone() {
    if (cubeCone) {
      setPrupMethod();
    } else {
      setYellowMethod();
    }
  }

  public void setError(double nextErr) {
    alignError = nextErr;
  }

  public void setAlignLED() {
    double maxAlignError = 3; // random value supposed to be max align error (if error is less than max, prog
                              // bar will start to fill up)
    double percentAlign = Math.min(1, Math.max(0, maxAlignError - alignError) / maxAlignError); // represents how
                                                                                                // aligned it is (1 =
                                                                                                // perfectly aligned , 0
                                                                                                // = not even close)

    int startLeft = 0;
    int startRight = 80;
    int endLeft = 40;
    int endRight = 120;
    int len = endLeft - startLeft;

    if (percentAlign > .9) {
      candle.animate(green);
    } else {
      if (cubeCone) {
        candle.setLEDs(255, 0, 255, 20, startLeft, (int) (len * percentAlign));
        candle.setLEDs(255, 0, 255, 20, startRight, (int) (len * percentAlign));
      } else {
        candle.setLEDs(255, 0, 255, 20, startLeft, (int) (len * percentAlign));
        candle.setLEDs(200, 150, 0, 0, startRight, (int) (len * percentAlign));
      }
      candle.setLEDs(0, 0, 0, 0, startLeft + (int) (len * percentAlign),
          endLeft - startLeft + (int) (len * percentAlign));
      candle.setLEDs(0, 0, 0, 0, startRight + (int) (len * percentAlign),
          endRight - startRight + (int) (len * percentAlign));
    }
  }

  @Override
  public void periodic() {
    // you have to compare prev network tables state to new one to see if it changed
    // because
    // CANdle cannot be set periodically or else it will lag
    // if(prevCubeCone != cubeCone){
    // updateLEDCubeCone();
    // // System.out.println("UPDATED TO: " + cubeCone);
    // }
    // prevCubeCone = cubeCone;
    // candle.get
    if (leveling) {
      candle.animate(whileLeveling);
    } else if (aligning) {
      // setAlignLED();
      if (finisehdAligning) {
        
        candle.animate(greenFin);
      }
      candle.animate(green);
    } else {

      if (cubeCone) {
        setPrupMethod();
      } else {
        setYellowMethod();
      }
    }
  
  }
}

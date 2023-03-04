// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleLEDSystem extends SubsystemBase {
  private final int ledCount = 65;
  /** true for cube, false for cone */
  boolean roborio = true;
  boolean purpleYellow = true;
  boolean prevpurpleYellow = true;

  AddressableLED leds;
  AddressableLEDBuffer ledBuffer;

 
  /** Creates a new CANdleLEDSystem. */
  public CANdleLEDSystem() {
   
   
    leds = new AddressableLED(3);
    ledBuffer = new AddressableLEDBuffer(115);

    leds.setLength(ledBuffer.getLength());
    leds.start();
    
  }
  

  //George Was here
  private void updateLEDCubeCone() {
    //George coded this
    if (purpleYellow) {
     setColor(200, 0, 200);
    }
    else {
     setColor(200, 150, 5);
    }


  }

  public void updatePurpYellow(){
    // System.out.println("PRUPR YELLOW UPDATE");
    purpleYellow = !purpleYellow;
  }

  private void setColor(int r, int g, int b) {
   
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, r, g, b);
      }
      leds.setData(ledBuffer);
    
  }
  @Override
  public void periodic() {
    if (prevpurpleYellow != purpleYellow) {
      updateLEDCubeCone();
    }
    prevpurpleYellow = purpleYellow;
    // System.out.println("curr color: " + purpleYellow);
    //you have to compare prev network tables state to new one to see if it changed because 
    //CANdle cannot be set periodically or else it will lag
    // if(prevCubeCone != cubeCone.getAsBoolean()){
    //   updateLEDCubeCone();
    // }
    // prevCubeCone = cubeCone.getAsBoolean();
   
  }
}
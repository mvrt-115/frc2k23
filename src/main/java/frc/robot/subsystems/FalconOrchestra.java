// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TalonFactory;

public class FalconOrchestra extends SubsystemBase {
  int[] ids = {1, 2, 3, 4, 5, 6, 7, 8};
  TalonFX[] talons;
  private Orchestra music;
  private boolean play = false;
  private String[] names = {
    "GoldenWind",
    "i-will-derive",
    "ImperialMarch",
    "Undertale",
    "WiiTheme"
  };
  /** Creates a new FalconOrchestra. */
  public FalconOrchestra() {
    music = new Orchestra();
    talons = new TalonFX[ids.length];
    for (int i = 0; i < ids.length; i++) {
      talons[i] = TalonFactory.createTalonFX(ids[i], false);
      music.addInstrument(talons[i]);
    }
    setMusic(names[2]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (play) {
      music.stop();
      music.play();
      play = false;
    }
  }

  public void playMusic() {
    play = true;
  }

  public void setMusic(String filename) {
    music.loadMusic(filename + ".chrp");
  }

  public String[] getRepertoire() {
    return this.names;
  }
}

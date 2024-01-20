// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class Lights extends SubsystemBase {

  private static Spark m_blinkin;

  /** Creates a new Lights. */
  public Lights() {

    //CAG: Need to update this with lights from 2024 PreSeason

    m_blinkin = new Spark(9);
    m_blinkin.set(.93);
  }

  public void changeColor(double value) {
    if (Math.abs(value) <= 1.0) {
      m_blinkin.set(value);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

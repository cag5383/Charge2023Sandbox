// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  
  private CANSparkMax intakeMotor;
  private CANSparkMax intakeMotorFollower;
//private beamBreakSensor?

  /** Creates a new Arm 💪 */
  public Intake() {

    intakeMotor = new CANSparkMax(PortConstants.kIntakeMotorPort, MotorType.kBrushless);
    intakeMotorFollower = new CANSparkMax(PortConstants.kIntakeMotorFollowerPort, MotorType.kBrushless);
    intakeMotorFollower.follow(intakeMotor, true);

    //CAG: do we need encoders if we're just maintaining speeds? Probably not?
  }
  

  public void runIntakeMotor() {
    // run the motor that changes the angle of the arm
    intakeMotor.set(IntakeConstants.intakeMotorSpeed);
  }

  public double getAngleMotorSpeed() {
    return intakeMotor.get();
  }

  public boolean getBeamBreak() {
//    CAG: return BeamBreakSensor.get();
    return true;
  }

  @Override
  public void periodic() {
    // set the arm's angle to 0 when we hit the limit switch

    if (getBeamBreak()) {
      intakeMotor.set(0);
      // CAG: Set lights to Orange
    }

    SmartDashboard.putBoolean("Has Note", getBeamBreak());
  }
}
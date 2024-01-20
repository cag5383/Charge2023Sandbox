// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooterMotor;
  private CANSparkMax shooterMotorFollower;
  // private beamBreakSensor?

  /** Creates a new Arm 💪 */
  public Shooter() {

    shooterMotor = new CANSparkMax(PortConstants.kShooterMotorPort, MotorType.kBrushless);
    shooterMotorFollower = new CANSparkMax(PortConstants.kShooterMotorFollowerPort, MotorType.kBrushless);
    shooterMotorFollower.follow(shooterMotor, true);

    // CAG: do we need encoders if we're just maintaining speeds? Probably not?
  }

  public void stopShootereMotor() {
    shooterMotor.set(0);
  }

  public void runShooterMotorFast() {
    // run the motor that changes the angle of the arm
    shooterMotor.set(ShooterConstants.kShooterFastSpeed);
  }

  public void runShooterMotorSlow() {
    // run the motor that changes the angle of the arm
    shooterMotor.set(ShooterConstants.kShooterSlowSpeed);
  }

  public double getShooterMotorSpeed() {
    return shooterMotor.get();
  }

  @Override
  public void periodic() {
  }
}
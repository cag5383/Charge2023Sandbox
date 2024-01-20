// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.DriveConstants.*;

public class DriveManuallyArcade extends CommandBase {
  /** Creates a new ArcadeDriveManually. */

  private final Drivetrain m_drivetrain;

  private final DoubleSupplier m_straightSpeed;
  private final DoubleSupplier m_turnSpeed;

  private double m_currentSpeed;

  public DriveManuallyArcade(boolean slowMode, DoubleSupplier straightSpeed, DoubleSupplier turnSpeed, Drivetrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    if (slowMode) {
      m_straightSpeed = () -> (straightSpeed.getAsDouble() * kSlowModeMultiplier);
    } else {
      m_straightSpeed = straightSpeed;
    }
    m_turnSpeed = turnSpeed;

    m_drivetrain = subsystem;

    addRequirements(m_drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_straightSpeed.getAsDouble(), m_turnSpeed.getAsDouble(), true);

    m_currentSpeed = m_drivetrain.getAverageVelocity();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_currentSpeed = 0;
    m_drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
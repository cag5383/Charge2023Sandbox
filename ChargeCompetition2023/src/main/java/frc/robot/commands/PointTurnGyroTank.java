// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;




public class PointTurnGyroTank extends CommandBase {

  private final Drivetrain m_drivetrain;

  private final double m_speed;

  private final double m_angle;

  
  /** Creates a new GyroTankPointTurn. */
  public PointTurnGyroTank(double speed, double angle, Drivetrain drivetrain) {

    m_speed = speed;
    m_drivetrain = drivetrain;
    m_angle = angle;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     m_drivetrain.calibrategyro();
     m_drivetrain.resetGyro();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrain.tankDrive(m_speed * Math.signum(m_angle), m_speed * Math.signum(m_angle) * -1, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_drivetrain.resetGyro();

    return Math.abs(m_drivetrain.getRobotAngle()) >= Math.abs(m_angle);
  }
}
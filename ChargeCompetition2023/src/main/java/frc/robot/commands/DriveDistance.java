// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
  /** Creates a new TankDriveDistance. */
  private final Drivetrain m_drivetrain; 
  private final double m_speed;
  private final double m_distance;


  public DriveDistance(Drivetrain subsystem, double speed, double distance) {
   // Use addRequirements() here to declare subsystem dependencies.
   
   m_drivetrain = subsystem;
   m_speed = speed; 
   m_distance = distance;
   addRequirements(m_drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // initializes and resets all encoders
    // m_drivetrain.encoderInit();
    m_drivetrain.resetAllEncoders();

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // sets the tank drive speeds to the inputed speeds
    m_drivetrain.tankDrive(m_speed, m_speed, false);   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDrive();
  }
    

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Command is finished when the travelled distance is greater than or equal to the inputted distance
   return Math.abs(m_drivetrain.getAverageDistance()) >= Math.abs(m_distance);
  }
}
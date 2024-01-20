package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import static frc.robot.Constants.VisionConstants.*;

public class VisionAlignShelf extends CommandBase {
  private PIDController m_distancePID;
  private PIDController m_anglePID;
  private double m_distanceOutput;
  private double m_angleOutput;
  private Vision m_vision;
  private Drivetrain m_drivetrain;

  public VisionAlignShelf(Vision vision, Drivetrain drivetrain) {
    m_vision = vision;
    m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_vision);
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_distancePID = new PIDController(kAutoAlignDistanceP, kAutoAlignDistanceI, kAutoAlignDistanceD);
    m_anglePID = new PIDController(kAutoAlignAngleP, kAutoAlignAngleI, kAutoAlignAngleD);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_distanceOutput = m_distancePID.calculate(m_vision.getYOffset(), kShelfAlignY);
    m_angleOutput = m_anglePID.calculate(m_vision.getXOffset(), 0);

    double leftMotorOutput = m_distanceOutput + m_angleOutput;
    double rightMotorOutput = m_distanceOutput - m_angleOutput;

    // potentially put clamps on each individual controller and not their sums
    if (leftMotorOutput > kClamp)
      leftMotorOutput = kClamp;
    else if (leftMotorOutput < -kClamp)
      leftMotorOutput = -kClamp;

    if (rightMotorOutput > kClamp)
      rightMotorOutput = kClamp;
    else if (rightMotorOutput < -kClamp)
      rightMotorOutput = -kClamp;

    m_drivetrain.tankDrive(leftMotorOutput, rightMotorOutput, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import static frc.robot.Constants.ArmConstants.*;

public class ChangeArmAngle extends CommandBase {

  private Arm m_Arm;
  private PIDController pid;

  private double finalAngle;
  private SlewRateLimiter m_ArmRampFilter;

  /** Creates a new ChangeArmAngle. */
  public ChangeArmAngle(Arm subsystem, double degree) {

    m_Arm = subsystem;
    pid = new PIDController(m_Arm.getAnglePID()[0], m_Arm.getAnglePID()[1], m_Arm.getAnglePID()[2]);
    
    finalAngle = degree;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmRampFilter = new SlewRateLimiter(0.6);

    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // run the motor at a positive speed to rotate the arm upward, and negative to rotate downward
    // if else in one line
    double pidOut = pid.calculate(m_Arm.getArmAngle(), finalAngle);
    if (pidOut > kAngleMaxMotorSpeed) {
      pidOut = kAngleMaxMotorSpeed;
    } else if (pidOut < -kAngleMaxMotorSpeed) {
      pidOut = -kAngleMaxMotorSpeed;
    }
    if (0 < pidOut && pidOut < kAngleMinMotorSpeed) {
      pidOut = kAngleMinMotorSpeed;
    } else if (0 > pidOut && pidOut > -kAngleMinMotorSpeed) {
      pidOut = -kAngleMinMotorSpeed;
    }

    m_Arm.setAngleMotorSpeed( m_ArmRampFilter.calculate(-pidOut));
    // m_Arm.setAngdleMotorSpeed((finalAngle > 0) ? kAngleMaxMotorSpeed : -kAngleMaxMotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.setAngleMotorSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Arm.getMinLimitSwitch() && m_Arm.getAngleMotorSpeed() < 0) {
      return true;
    }
    if (m_Arm.getMaxLimitSwitch() && m_Arm.getAngleMotorSpeed() > 0) {
      return true;
    }
    // stop the motor after reaching the desired angle
    // return false;
    return (Math.abs(m_Arm.getArmAngle() - finalAngle) <= kArmAngleTolerance);
    
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.ClawConstants;

public class ClawPistonExtend extends InstantCommand {
  private final Claw m_claw;

  /** Creates a new PistonExtend. */
  public ClawPistonExtend(Claw subsystem) {
    m_claw = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_claw);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.extendRightPiston();
  }
}

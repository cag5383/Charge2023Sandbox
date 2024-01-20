// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.ClawConstants;

public class ClawEject extends CommandBase {
  private static final double kshootSpeed = -.9; //-.75
  private static final double kCubeShootSpeed = 0;
  private static final double kConeShootSpeed = 0;
  private final Claw m_claw;

  private double startTime;

  /** Creates a new ClawPickup. */
  public ClawEject(Claw subsystem) {

    m_claw = subsystem;

    startTime = System.currentTimeMillis();

    // Use addRequirements() here to declare subsystem dependencies.

  addRequirements(m_claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Determines which game piece there is and then ejects the piece accordingly.
// if (m_claw.hasPieceCone()== true) {
//   m_claw.ejectConeRollerMotors(kConeShootSpeed);
// } else {
//   m_claw.ejectCubeRollerMotors(kCubeShootSpeed);
// }
     m_claw.runRollerMotors(kshootSpeed); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //after the game piece is ejected, the pistons return to the default position and motors are stopped.
    //if (!(m_claw.hasPieceCone()) || !(m_claw.hasPieceCube())) {
    //m_claw.extendLeftPiston();
    m_claw.extendRightPiston();
    m_claw.haltMotors(0);

    }
    
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((System.currentTimeMillis() - startTime) > 1250);
  }
}
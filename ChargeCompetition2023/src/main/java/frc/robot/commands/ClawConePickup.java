// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawConePickup extends CommandBase {
  private static final double kRollerMotorConeStopSpeed = 0;
  private final Claw m_claw;

  /** Creates a new ClawConePickup. */
  public ClawConePickup(Claw subsystem) {

    m_claw = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.

  addRequirements(m_claw);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
    //initiates the cone pickup sequence by extending the pistons and then running the motors. The motors will then stop after the claw has a "grip" on the cone (stopRollerMotors is called)

  public void execute() {
//the pistons (which are connected to one roller) close in to squeeze the cone. *the retract values (located in claw constants) may need to be tweaked during testing to see which length is the correct length for the cone pickup.
//m_claw.retractLeftPiston();
m_claw.extendRightPiston();
m_claw.runRollerMotors(0.25); //the motors spin to intake the cone upwards
//m_claw.stopRollerMotorsCone(kRollerMotorConeStopSpeed); //once the current limit is reached, this should mean that the cone is being held and the motors are ready to stop. 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //motors are completely stopped
    m_claw.haltMotors(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ChangeArmAngle;
import frc.robot.subsystems.Arm;

/* 
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Wait;
import frc.robot.subsystems.Claw;
import frc.robot.commands.ChangeArmAngle;
import frc.robot.commands.ClawEject;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.PIDbalancechargestation;
*/

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootHigh extends SequentialCommandGroup {
  /** Creates a new AutoScoreCubeInMiddleAndEngage. */
  public ShootHigh(Arm arm) {

    // Adjust the arm
    addCommands(new ChangeArmAngle(arm, ArmConstants.kSpeakerAngle));
    // Get shooting motor up to speed
    // Run the intake motors for a second
    // Turn off the shooting motor?
  }
}

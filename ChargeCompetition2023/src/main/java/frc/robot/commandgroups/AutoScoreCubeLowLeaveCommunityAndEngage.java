// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ChangeArmAngle;
import frc.robot.commands.ClawEject;
import frc.robot.commands.ClawPistonRetract;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.PIDbalancechargestation;
import frc.robot.commands.Wait;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
// import frc.robot.subsystems.Vision;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreCubeLowLeaveCommunityAndEngage extends SequentialCommandGroup {
  /** Creates a new AutoScoreCubeInMiddleAndEngage. */
  public AutoScoreCubeLowLeaveCommunityAndEngage(Drivetrain drivetrain, Arm arm, Claw claw) {
    
    // addCommands(new ChangeArmAngle(arm, ArmConstants.kShelfAngle)) 
    // addCommands(new VisionAlignToRetroreflectiveTape(vision, drivetrain));

    addCommands(new ClawPistonRetract(claw));
    addCommands(new ClawEject(claw));
    addCommands(new DriveDistance(drivetrain, 0.3, DriveConstants.kDistanceOverChargeStation));
    addCommands(new Wait(250));
    addCommands(new DriveDistance(drivetrain, -0.3, DriveConstants.KDistanceReturnChargeStation));
    addCommands(new Wait(500));
    addCommands(new PIDbalancechargestation(drivetrain));

  }
}
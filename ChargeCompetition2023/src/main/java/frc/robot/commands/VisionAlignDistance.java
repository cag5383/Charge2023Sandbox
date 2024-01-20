// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.VisionConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionAlignDistance extends PIDCommand {

  private final Vision m_vision;
  private final Drivetrain m_drivetrain;

  public VisionAlignDistance(Vision vision, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(kAutoAlignDistanceP, kAutoAlignDistanceI, kAutoAlignDistanceD),
        // This should return the measurement
        () -> vision.getYOffset(),
        // This should return the setpoint (can also be a constant)
        () -> kShelfAlignY,
        // This uses the output
        output -> {
          // Use the output here

          drivetrain.tankDrive(output, output, false);
        });

        m_vision = vision;
        m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_vision);
    addRequirements(m_drivetrain);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DriveConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PointTurnGyroPID extends PIDCommand {
  /** Creates a new PointTurnEncoderPID. */  

  public PointTurnGyroPID(double angle, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(kTurnP, kTurnI, kTurnD),
        // This should return the measurement
        () -> drivetrain.getRobotAngle(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          // Use the output here
          double o = MathUtil.clamp(Math.abs(output), 0.1, 0.5);
          drivetrain.tankDrive(o * Math.signum(output), -o * Math.signum(output), false);
  
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);

    drivetrain.resetGyro();

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
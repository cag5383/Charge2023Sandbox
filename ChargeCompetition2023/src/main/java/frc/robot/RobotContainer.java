// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final XboxController m_driver = new
  // XboxController(DriveConstants.kDriverControllerPort);
  // public final XboxController m_operator = new
  // XboxController(DriveConstants.kOperatorControllerPort);
  private Drivetrain m_drivetrain;
  private Arm m_arm;
  private Intake m_intake;
  private Lights m_lights;
  private Vision m_vision;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // initialize subsystems
    initSubsystems();

    // Configure the button bindings
    configureDriverButtonBindings();
    configureOperatorButtonBindings();

    // Set Default Commands
    setDefaultCommands();
  }

  private boolean initSubsystems() {
    if (SubsystemConstants.kUseDriveTrain) {
      m_drivetrain = new Drivetrain();
    }

    if (SubsystemConstants.kUseArm) {
      m_arm = new Arm();
    }

    if (SubsystemConstants.kUseIntake) {
      m_intake = new Intake();
    }

    if (SubsystemConstants.kUseLights) {
      m_lights = new Lights();
    }

    if (SubsystemConstants.kUseVision) {
      m_vision = new Vision();
    }

    return true;
  }

  private boolean setDefaultCommands() {
    // CAG if there is a command a subsystem should be running at all times, this is
    // where we would set it.
    // For example, the drivetrain should always be reading driver inputs and
    // running some kind of drive command.

    // m_drivetrain.setDefaultCommand(
    // new DriveManuallyArcade(m_drivetrain.isSlow(), () -> -m_driver.getLeftY() *
    // normalspeed, () -> -m_driver.getRightX()* normalspeed, m_drivetrain));
    // new JoystickButton(m_driver, Button.kLeftBumper.value).whileTrue(new
    // DriveManuallyArcade(true, () -> -m_driver.getLeftY() * normalspeed, () ->
    // -m_driver.getRightX()* normalspeed, m_drivetrain));

    return true;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureDriverButtonBindings() {
    /*
     * // new JoystickButton(m_operator, Button.kB.value).whileTrue(
     * // new ChangeLedColor(lights, 0.57));
     * 
     * // green
     * // new JoystickButton(m_operator, Button.kY.value).whileTrue(
     * // new ChangeLedColor(lights, 0.81));
     * 
     * // yellow
     * // new JoystickButton(m_operator, Button.kX.value).whileTrue(
     * // new ChangeLedColor(lights, 0.69));
     * 
     * // violet
     * // new JoystickButton(m_operator, Button.kA.value).whileTrue(
     * // new ChangeLedColor(lights, 0.91));
     * 
     * // new JoystickButton(m_driver, Button.kStart.value).toggleOnTrue(new
     * // DriveManuallyArcade(() -> -m_driver.getLeftY() * slowspeed, () ->
     * // -m_driver.getRightX()* slowspeed, m_drivetrain));
     * 
     * new JoystickButton(m_operator, Button.kA.value).whileTrue(
     * new ChangeArmManually(arm, -1));
     * 
     * new JoystickButton(m_operator, Button.kB.value).whileTrue(
     * new ChangeArmManually(arm, 1));
     * 
     * new JoystickButton(m_operator, Button.kX.value).whileTrue(
     * new ChangeArmAngle(arm, 12)); //chute
     * 
     * // new JoystickButton(m_operator, Button.kY.value).whileTrue(
     * // new ChangeArmAngle(arm, kShelfAngle)); //195
     * 
     * new JoystickButton(m_operator, Button.kY.value).whileTrue(
     * new ChangeArmAngle(arm, 37));
     * 
     * new JoystickButton(m_operator, Button.kBack.value).whileTrue(
     * new ChangeArmAngle(arm, 196)); //cone
     * 
     * new Trigger(() ->(m_operator.getRightTriggerAxis()>0.2)).whileTrue(
     * new ChangeArmAngle(arm, 0));
     * 
     * // 215:mid cone
     * // 195:shelf
     * // Schedule `exampleMethodCommand` when the Xbox controller's B button is
     * pressed,
     * // cancelling on release.
     * new JoystickButton(m_driver, Button.kX.value).whileTrue(
     * new PIDbalancechargestation(m_drivetrain));
     * 
     * new JoystickButton(m_driver, Button.kA.value).whileTrue(
     * new VisionAlignDistanceAndAngle(m_vision, m_drivetrain,kShelfAlignY,0));
     * 
     * new JoystickButton(m_driver, Button.kB.value).whileTrue(
     * new VisionAlignDistanceAndAngle(m_vision, m_drivetrain,kChuteAlignY,0));
     * 
     * new JoystickButton(m_driver, Button.kY.value).whileTrue(
     * new VisionAlignShelf(m_vision, m_drivetrain));
     * 
     * new JoystickButton(m_operator, Button.kRightStick.value).whileTrue(new
     * ClawEject(claw));
     * new JoystickButton(m_operator, Button.kLeftStick.value).whileTrue(new
     * ClawEject(claw));
     * 
     * //rBumpOpButton
     * new JoystickButton(m_operator, Button.kRightBumper.value).whileTrue(new
     * ClawCubePickup(claw));
     * 
     * new JoystickButton(m_operator, Button.kStart.value).whileTrue(new
     * ChangeArmAngle(arm, 202)); //cube
     * 
     * new JoystickButton(m_operator, Button.kLeftBumper.value).whileTrue(new
     * ClawConePickup(claw));
     */
  }

    private void configureOperatorButtonBindings() {
    }
}
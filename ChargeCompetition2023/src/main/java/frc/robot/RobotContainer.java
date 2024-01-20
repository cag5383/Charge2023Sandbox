// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ArmConstants.kMidConeAngle;
import static frc.robot.Constants.ArmConstants.kShelfAngle;

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
import frc.robot.Constants.DriveConstants;
import frc.robot.commandgroups.AutoScoreConeInMiddleAndEngage;
import frc.robot.commandgroups.AutoScoreConeInMiddleLeaveCommunityAndEngage;
import frc.robot.commandgroups.AutoScoreConeOnSideAndDriveBackwards;
import frc.robot.commandgroups.AutoScoreConeOnSideAndDriveToSubstation;
import frc.robot.commandgroups.AutoScoreCubeInLowAndEngage;
import frc.robot.commandgroups.AutoScoreCubeInMiddleAndEngage;
import frc.robot.commandgroups.AutoScoreCubeInMiddleLeaveCommunityAndEngage;
import frc.robot.commandgroups.AutoScoreCubeLowLeaveCommunityAndEngage;
import frc.robot.commandgroups.AutoScoreCubeOnSideAndDriveBackwards;
import frc.robot.commandgroups.AutoScoreCubeOnSideAndDriveToSubstation;
import frc.robot.commandgroups.DriveBackwardsAndBalance;
import frc.robot.commands.ChangeArmAngle;
import frc.robot.commands.ChangeArmManually;
import frc.robot.commands.ClawConePickup;
import frc.robot.commands.ClawCubePickup;
import frc.robot.commands.ClawEject;
import frc.robot.commands.DriveManuallyArcade;
import frc.robot.commands.JoystickMoveArm;
import frc.robot.commands.PIDbalancechargestation;
import frc.robot.commands.VisionAlignAngle;
import frc.robot.commands.VisionAlignDistance;
import frc.robot.commands.VisionAlignDistanceAndAngle;
import frc.robot.commands.VisionAlignShelf;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import static frc.robot.Constants.VisionConstants.*;

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
  // private static final double slowspeed = 0.5;
  // The robot's subsystems and commands are defined here...
  private final XboxController m_driver = new XboxController(DriveConstants.kDriverControllerPort);
  public final XboxController m_operator = new XboxController(DriveConstants.kOperatorControllerPort);

  private final Claw claw;
  private final Drivetrain m_drivetrain;
  private final Arm arm;
  // private final Lights lights;

  // private final Index m_index;
  // private final Intake m_intake;
  // private final Shooter m_shooter;
  private final Vision m_vision;
  // private final Compressor m_testCompressor;

  private final SendableChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain = new Drivetrain();
    arm = new Arm();
    claw = new Claw();
    double normalspeed = .75;

    // m_index = new Index();
    // m_intake = new Intake();

    m_vision = new Vision();
    // m_shooter = new Shooter();

    m_autoChooser = new SendableChooser<>();
    SmartDashboard.putData("Autonomous Selector", m_autoChooser);
    m_autoChooser.setDefaultOption("Do Nothing", new InstantCommand());

    //m_autoChooser.addOption("DriveBackwardsAndBalance", new DriveBackwardsAndBalance(m_drivetrain));
    //m_autoChooser.addOption("Score Cone on Side and Leave Community", new AutoScoreConeOnSideAndDriveBackwards(m_drivetrain, arm, claw));
    m_autoChooser.addOption("Score Low Cube on Side and Leave Community", new AutoScoreCubeOnSideAndDriveBackwards(m_drivetrain, arm, claw));
   // m_autoChooser.addOption("Score Cone on Side and Drive to Substation", new AutoScoreConeOnSideAndDriveToSubstation(m_drivetrain, arm, claw));
    //m_autoChooser.addOption("Score Cube on Side and Drive to Substation", new AutoScoreCubeOnSideAndDriveToSubstation(m_drivetrain, arm, claw));
    //m_autoChooser.addOption("Score Cone in Middle and Engage", new AutoScoreConeInMiddleAndEngage(m_drivetrain, arm, claw));
    m_autoChooser.addOption("Mid Nova", new AutoScoreCubeInMiddleAndEngage(m_drivetrain, arm, claw));
    m_autoChooser.addOption("Low Nova", new AutoScoreCubeInLowAndEngage(m_drivetrain, arm, claw));
    //m_autoChooser.addOption("Score Cone in Middle, Leave Community, and Engage", new AutoScoreConeInMiddleLeaveCommunityAndEngage(m_drivetrain, arm, claw));
    m_autoChooser.addOption("Mid SuperNova", new AutoScoreCubeInMiddleLeaveCommunityAndEngage(m_drivetrain, arm, claw));
    m_autoChooser.addOption("Low SuperNova", new AutoScoreCubeLowLeaveCommunityAndEngage(m_drivetrain, arm, claw));

    
    m_drivetrain.setDefaultCommand(
      new DriveManuallyArcade(m_drivetrain.isSlow(), () -> -m_driver.getLeftY() * normalspeed, () -> -m_driver.getRightX()* normalspeed, m_drivetrain));
      new JoystickButton(m_driver, Button.kLeftBumper.value).whileTrue(new DriveManuallyArcade(true,  () -> -m_driver.getLeftY() * normalspeed, () -> -m_driver.getRightX()* normalspeed, m_drivetrain));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {

    // new JoystickButton(m_operator, Button.kB.value).whileTrue(
    // new ChangeLedColor(lights, 0.57));

    //  green
    // new JoystickButton(m_operator, Button.kY.value).whileTrue(
    // new ChangeLedColor(lights, 0.81));

    // yellow
    // new JoystickButton(m_operator, Button.kX.value).whileTrue(
    // new ChangeLedColor(lights, 0.69));

    // violet
    // new JoystickButton(m_operator, Button.kA.value).whileTrue(
    // new ChangeLedColor(lights, 0.91));

    // new JoystickButton(m_driver, Button.kStart.value).toggleOnTrue(new
    // DriveManuallyArcade(() -> -m_driver.getLeftY() * slowspeed, () ->
    // -m_driver.getRightX()* slowspeed, m_drivetrain));

    new JoystickButton(m_operator, Button.kA.value).whileTrue(
        new ChangeArmManually(arm, -1));

    new JoystickButton(m_operator, Button.kB.value).whileTrue(
        new ChangeArmManually(arm, 1));

    new JoystickButton(m_operator, Button.kX.value).whileTrue(
        new ChangeArmAngle(arm, 12)); //chute
 
    // new JoystickButton(m_operator, Button.kY.value).whileTrue(
    //     new ChangeArmAngle(arm, kShelfAngle)); //195

    new JoystickButton(m_operator, Button.kY.value).whileTrue(
        new ChangeArmAngle(arm,  37));
 
    new JoystickButton(m_operator, Button.kBack.value).whileTrue(
      new ChangeArmAngle(arm, 196)); //cone

    new Trigger(() ->(m_operator.getRightTriggerAxis()>0.2)).whileTrue(
      new ChangeArmAngle(arm, 0));

          // 215:mid cone
          // 195:shelf
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    new JoystickButton(m_driver, Button.kX.value).whileTrue(
      new PIDbalancechargestation(m_drivetrain));
      
    new JoystickButton(m_driver, Button.kA.value).whileTrue(
      new VisionAlignDistanceAndAngle(m_vision, m_drivetrain,kShelfAlignY,0));

      new JoystickButton(m_driver, Button.kB.value).whileTrue(
        new VisionAlignDistanceAndAngle(m_vision, m_drivetrain,kChuteAlignY,0));

     new JoystickButton(m_driver, Button.kY.value).whileTrue(
      new VisionAlignShelf(m_vision, m_drivetrain));

    new JoystickButton(m_operator, Button.kRightStick.value).whileTrue(new ClawEject(claw));
    new JoystickButton(m_operator, Button.kLeftStick.value).whileTrue(new ClawEject(claw)); 

  //rBumpOpButton
 new JoystickButton(m_operator, Button.kRightBumper.value).whileTrue(new ClawCubePickup(claw));

 new JoystickButton(m_operator, Button.kStart.value).whileTrue(new ChangeArmAngle(arm, 202)); //cube

    new JoystickButton(m_operator, Button.kLeftBumper.value).whileTrue(new ClawConePickup(claw));
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
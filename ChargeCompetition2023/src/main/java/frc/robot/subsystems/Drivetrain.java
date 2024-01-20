// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.PortConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  /** Creates a new Drivetrain. */

  // Declare Motor Variables
  private final CANSparkMax m_leftLeaderMotor;
  private final CANSparkMax m_rightLeaderMotor;
  private final CANSparkMax m_leftFollowerMotor;
  private final CANSparkMax m_rightFollowerMotor;

  // Declare Encoder Variables
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final DifferentialDrive m_drive;

  //private final DifferentialDriveKinematics m_kinematics;
 private final SlewRateLimiter m_RampFilter;

 private boolean slowMode;


  public Drivetrain() {

    // Set Motor Ports and Motor Type
    m_leftLeaderMotor = new CANSparkMax(PortConstants.kLeftLeaderMotorPort, MotorType.kBrushless);
    m_rightLeaderMotor = new CANSparkMax(PortConstants.kRightLeaderMotorPort, MotorType.kBrushless);
    m_leftFollowerMotor = new CANSparkMax(PortConstants.kLeftFollowerMotorPort, MotorType.kBrushless);
    m_rightFollowerMotor = new CANSparkMax(PortConstants.kRightFollowerMotorPort, MotorType.kBrushless);

    // Initialize Motors
    motorInit(m_leftLeaderMotor, DriveConstants.kLeftLeaderMotorReversedDefault);
    motorInit(m_rightLeaderMotor, DriveConstants.kRightLeaderMotorReversedDefault);
    motorInit(m_leftFollowerMotor, DriveConstants.kLeftFollowerMotorReversedDefault);
    motorInit(m_rightFollowerMotor, DriveConstants.kRightFollowerMotorReversedDefault);

    // Set Motor Followers
    m_leftFollowerMotor.follow(m_leftLeaderMotor);
    m_rightFollowerMotor.follow(m_rightLeaderMotor);

    // Set Encoder to Leader Motor Encoder
    m_leftEncoder = m_leftLeaderMotor.getEncoder();
    m_rightEncoder = m_rightLeaderMotor.getEncoder();

    encoderInit();
    resetAllEncoders();

    slowMode = false;

    m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);
    m_RampFilter = new SlewRateLimiter(0.8);
  

    //m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putNumber("left drive encoder", m_leftEncoder.getPosition());
     SmartDashboard.putNumber("right drive encoder", m_rightEncoder.getPosition());
     SmartDashboard.putNumber("average distance", getAverageDistance());
    // 3.14 is to compensate for the battery and the floor, might have to change that at a later time
    SmartDashboard.putBoolean("Slow Mode Enabled", isSlow());
  
  }

  // Initializes Motors by Setting Defaults
  private void motorInit(CANSparkMax motor, boolean invert) {
    motor.restoreFactoryDefaults(); 
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
    motor.setInverted(invert);
  }

  public void encoderInit() {
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kDriveDistancePerRev);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kDriveSpeedPerRev);

    m_rightEncoder.setPositionConversionFactor(DriveConstants.kDriveDistancePerRev);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kDriveSpeedPerRev);
  }
  // Reset Encoder
  private void encoderReset(RelativeEncoder encoder){
    encoder.setPosition(0);
  }

  //Reset Left Encoder
   public void leftEncoderReset(){
   encoderReset(m_leftEncoder);
   }

   // Reset Right Encoder
 public void rightEncoderReset(){
     encoderReset(m_rightEncoder);
   }

  // // Reset all Encoders 
 public void resetAllEncoders(){
    encoderReset(m_leftEncoder);
    encoderReset(m_rightEncoder);
 }

  // // Get the position of the left encoders
  public double getLeftDistance(){
   return m_leftEncoder.getPosition();
  }

  // // Get the position of the right encoders
   public double getRightDistance(){
   return m_rightEncoder.getPosition();
   }

  // Averages the left and right encoder distance
   public double getAverageDistance(){
   double distLeft = getLeftDistance();
   double distRight = getRightDistance();
   
 encoderInit();
  //printPositionConversionFactor();   
   
  //System.out.println("Left:  " + distLeft);
  // System.out.println("Right:  " + distRight);
  //  System.out.println("velocity:  " + getAverageVelocity());
     
   return (distLeft + distRight) / 2;

 }

  // // feedback of encoder conversion factor on the Driver Station Console
 public void printPositionConversionFactor() {
   //System.out.println("Left Conversion Factor:  " + m_leftEncoder.getPositionConversionFactor());
   // System.out.println("Right Conversion Factor:  " + m_rightEncoder.getPositionConversionFactor());
   }

  // // Get the velocity of the left encoder
   private double getLeftSpeed(){
    return m_leftEncoder.getVelocity();
   }

  // // Get the velocity of the right encoder
  private double getRightSpeed(){
    return m_rightEncoder.getVelocity();
   }

   // Get the average velocity
   public double getAverageVelocity(){
    return (getLeftSpeed() + getRightSpeed()) / 2;
   }


  // Drives Using Tank Drive
  public void tankDrive(double leftPower, double rightPower, boolean squareInputs){
    // double outputLeft = m_leftPIDController.calculate(getLeftSpeed(), setpoint);
    // double outputRight = pid.calculate(encoder.getDistance(), setpoint);
    m_drive.tankDrive(leftPower, rightPower, squareInputs);

  }
  
  // Drives Using Arcade Drive
  public void arcadeDrive(double speed, double turn, boolean squareInputs){
    m_drive.arcadeDrive(m_RampFilter.calculate(speed), turn, squareInputs);

    // SmartDashboard.putNumber("forward power", speed);

  }
  // Stop All Drive Motors
  public void stopDrive(){
    m_drive.tankDrive(0, 0);
  }
  
  // public void toggleSlowMode() {
  //   slowMode = !slowMode;
  // }

  public boolean isSlow() {
    return slowMode;
  }

}
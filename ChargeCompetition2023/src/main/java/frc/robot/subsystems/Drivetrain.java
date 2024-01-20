// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.kCurrentLimit;
import static frc.robot.Constants.DriveConstants.kDriveDistancePerRev;
import static frc.robot.Constants.DriveConstants.kDriveSpeedPerRev;
import static frc.robot.Constants.DriveConstants.kLeftA;
import static frc.robot.Constants.DriveConstants.kLeftFollowerMotorPort;
import static frc.robot.Constants.DriveConstants.kLeftFollowerMotorReversedDefault;
import static frc.robot.Constants.DriveConstants.kLeftLeaderMotorPort;
import static frc.robot.Constants.DriveConstants.kLeftLeaderMotorReversedDefault;
import static frc.robot.Constants.DriveConstants.kLeftS;
import static frc.robot.Constants.DriveConstants.kLeftV;
import static frc.robot.Constants.DriveConstants.kRightA;
import static frc.robot.Constants.DriveConstants.kRightFollowerMotorPort;
import static frc.robot.Constants.DriveConstants.kRightFollowerMotorReversedDefault;
import static frc.robot.Constants.DriveConstants.kRightLeaderMotorPort;
import static frc.robot.Constants.DriveConstants.kRightLeaderMotorReversedDefault;
import static frc.robot.Constants.DriveConstants.kRightS;
import static frc.robot.Constants.DriveConstants.kRightV;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
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



  private final SimpleMotorFeedforward m_leftFeedforward;
  private final SimpleMotorFeedforward m_rightFeedforward;



   private final ADIS16470_IMU m_imu;
  private final DifferentialDriveKinematics m_kinematics;
 private final SlewRateLimiter m_RampFilter;

 private boolean slowMode;


  public Drivetrain() {

    // Set Motor Ports and Motor Type
    m_leftLeaderMotor = new CANSparkMax(kLeftLeaderMotorPort, MotorType.kBrushless);
    m_rightLeaderMotor = new CANSparkMax(kRightLeaderMotorPort, MotorType.kBrushless);
    m_leftFollowerMotor = new CANSparkMax(kLeftFollowerMotorPort, MotorType.kBrushless);
    m_rightFollowerMotor = new CANSparkMax(kRightFollowerMotorPort, MotorType.kBrushless);

    // Initialize Motors
    motorInit(m_leftLeaderMotor, kLeftLeaderMotorReversedDefault);
    motorInit(m_rightLeaderMotor, kRightLeaderMotorReversedDefault);
    motorInit(m_leftFollowerMotor, kLeftFollowerMotorReversedDefault);
    motorInit(m_rightFollowerMotor, kRightFollowerMotorReversedDefault);

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

    // Set The Feedforward Values
    m_leftFeedforward = new SimpleMotorFeedforward(kLeftS, kLeftV, kLeftA);
    m_rightFeedforward = new SimpleMotorFeedforward(kRightS, kRightV, kRightA);
    


    m_imu = new ADIS16470_IMU();
    m_imu.calibrate();
 m_RampFilter = new SlewRateLimiter(0.8);
  

    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putNumber("left drive encoder", m_leftEncoder.getPosition());
     SmartDashboard.putNumber("right drive encoder", m_rightEncoder.getPosition());
     SmartDashboard.putNumber("average distance", getAverageDistance());
    // m_imu.setYawAxis(IMUAxis())
    // 3.14 is to compensate for the battery and the floor, might have to change that at a later time
    SmartDashboard.putNumber("GyroYaw", getRobotAngle());
    SmartDashboard.putNumber("GyroX", m_imu.getXComplementaryAngle() - 3.14);
    SmartDashboard.putBoolean("Slow Mode Enabled", isSlow());
    SmartDashboard.putNumber("Pitch", m_imu.getYComplementaryAngle());
  
  }

  // Initializes Motors by Setting Defaults
  private void motorInit(CANSparkMax motor, boolean invert) {
    motor.restoreFactoryDefaults(); 
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(kCurrentLimit);
    motor.setInverted(invert);
  }

  public void encoderInit() {
    m_leftEncoder.setPositionConversionFactor(kDriveDistancePerRev);
    m_leftEncoder.setVelocityConversionFactor(kDriveSpeedPerRev);

    m_rightEncoder.setPositionConversionFactor(kDriveDistancePerRev);
    m_rightEncoder.setVelocityConversionFactor(kDriveSpeedPerRev);
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
  
  public void resetGyro(){
  
    m_imu.reset();
  
  }
  public void calibrategyro() {
    m_imu.calibrate();
  }

  public double getRobotAngle(){
    return m_imu.getAngle();
  }

  public double getRollangle(){
    // return (m_imu.getXComplementaryAngle() * -1 );
    return (m_imu.getYComplementaryAngle());

  }

  // public void toggleSlowMode() {
  //   slowMode = !slowMode;
  // }

  public boolean isSlow() {
    return slowMode;
  }

}
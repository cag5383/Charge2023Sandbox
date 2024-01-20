// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants;
import static frc.robot.Constants.PortConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax angleMotor;
  private CANSparkMax angleMotorFollower;
  private RelativeEncoder angleEncoder;
  private DigitalInput minLimitSwitch;
  private DigitalInput maxLimitSwitch;

  private double[] angleMotorPID;
  private double setpoint;

  /** Creates a new Arm 💪 */
  public Arm() {

    angleMotor = new CANSparkMax(PortConstants.kAngleMotorPort, MotorType.kBrushless);
    angleEncoder = angleMotor.getEncoder();
    angleMotorFollower = new CANSparkMax(PortConstants.kAngleMotorFollowerPort, MotorType.kBrushless);
    angleMotorFollower.follow(angleMotor, true);
    angleMotorPID = new double[] { ArmConstants.kP, ArmConstants.kI, ArmConstants.kD };
    minLimitSwitch = new DigitalInput(PortConstants.kMinLimitSwitchPort);
    maxLimitSwitch = new DigitalInput(PortConstants.kMaxLimitSwitchPort);
    resetEncoders();
  }

  public void setAngleMotorSpeed(double speed) {
    // run the motor that changes the angle of the arm
    angleMotor.set(speed);
  }

  public void setArmAngle(double angle) {
    setpoint = angle;
    // CAG: Need a PID controller to get it to the right spot.
  }

  public boolean checkArmAngleDelta(){
    double delta = Math.abs(angleMotor.get() - setpoint);
    return (delta < ArmConstants.kArmAngleTolerance);
  }

  public double getAngleMotorSpeed() {
    return angleMotor.get();
  }

  public double getArmAngle() {
    // CAG: Why is this negative?
    return -angleEncoder.getPosition();
  }

  public boolean getMinLimitSwitch() {
    return minLimitSwitch.get();
  }

  public boolean getMaxLimitSwitch() {
    return maxLimitSwitch.get();
  }

  public void resetEncoders() {
    // reset the angleMotor's encoder
    angleEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // set the arm's angle to 0 when we hit the limit switch
    if (getMinLimitSwitch()) {
      // angleEncoder.setPosition(ArmConstants.kMaxAngle);
    }

    if (getMaxLimitSwitch()) {
      resetEncoders();
    }

    SmartDashboard.putBoolean("Rear Limit Switch", getMinLimitSwitch());
    SmartDashboard.putBoolean("Front Limit Switch", getMaxLimitSwitch());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle (deg)", getArmAngle());
    SmartDashboard.putNumber("Arm Speed", getAngleMotorSpeed());
  }
}
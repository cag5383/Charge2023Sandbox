// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
  
  private CANSparkMax angleMotor;
  private CANSparkMax angleMotorFollower;
  private RelativeEncoder angleEncoder;
  private DigitalInput minLimitSwitch;
  private DigitalInput maxLimitSwitch;

  private double[] angleMotorPID;


  /** Creates a new Arm 💪 */
  public Arm() {

    angleMotor = new CANSparkMax(kAngleMotorPort, MotorType.kBrushless);
    angleEncoder = angleMotor.getEncoder();

    angleMotorFollower = new CANSparkMax(kAngleMotorFollowerPort, MotorType.kBrushless);

    angleMotorFollower.follow(angleMotor, true);

    angleMotorPID = new double[]{kP,kI,kD};

    minLimitSwitch = new DigitalInput(kMinLimitSwitchPort);
    maxLimitSwitch = new DigitalInput(kMaxLimitSwitchPort);

    angleEncoder.setPositionConversionFactor(360/kEncoderTicksPerRevolution);
    

    resetEncoders();
  }
  

  public void setAngleMotorSpeed(double speed) {
    // run the motor that changes the angle of the arm
    angleMotor.set(speed);
  }

  public double getAngleMotorSpeed() {
    return angleMotor.get();
  }

  // public double getStringPotDistance() {
  //   // get the voltage read by the string potientometer in terms of distance
  //   return stringPot.get();
  // }

  public double getArmAngle() {
    // 0 degree = touching rear switch
    // 282 degrees (maximum) = touching front switch
    return -angleEncoder.getPosition();
  }

  public double[] getAnglePID() {
    return angleMotorPID;
  }

  public boolean getMinLimitSwitch() {
    // return minLimitSwitch.get();
    return false;
    // return getArmAngle() > 220;

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
      angleEncoder.setPosition(kMaxAngle);
    }

    if (getMaxLimitSwitch()) {
      resetEncoders();
    }

    SmartDashboard.putBoolean("Rear Limit Switch", getMinLimitSwitch());
    SmartDashboard.putBoolean("Front Limit Switch", getMaxLimitSwitch());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle (deg)", getArmAngle());
    SmartDashboard.putNumber("Arm Speed", getAngleMotorSpeed());
    

    // angleMotorPID[0] = SmartDashboard.getNumber("Arm Angle P", 0.0);
    // angleMotorPID[1] = SmartDashboard.getNumber("Arm Angle I", 0.0);
    // angleMotorPID[2] = SmartDashboard.getNumber("Arm Angle D", 0.0);

    // SmartDashboard.putString("Angle PID In Use", Arrays.toString(angleMotorPID));


  }
}
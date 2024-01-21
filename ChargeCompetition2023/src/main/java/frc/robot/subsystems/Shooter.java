// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Units;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooterMotor;
  private CANSparkMax shooterMotorFollower;

  // Shooter PID
  private double shooterSetpoint = 0;
  private SparkPIDController c_pid;

  public Shooter() {

    shooterMotor = new CANSparkMax(PortConstants.kShooterMotorPort, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    shooterMotorFollower = new CANSparkMax(PortConstants.kShooterMotorFollowerPort, MotorType.kBrushless);
    shooterMotorFollower.restoreFactoryDefaults();
    shooterMotorFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterMotorFollower.follow(shooterMotor, true);

    c_pid = shooterMotor.getPIDController();
    c_pid.setP(ShooterConstants.kP);
    c_pid.setI(ShooterConstants.kI);
    c_pid.setD(ShooterConstants.kD);
    c_pid.setFF(ShooterConstants.kFF);

    // CAG Make sure to be careful with units on these - sanity check controller
    // outputs to make sure it makes any kind of sense! I think they're not actually
    // RPM, but decimal% of max?
    c_pid.setOutputRange(ShooterConstants.minRPM, ShooterConstants.maxRPM);
    changeShooterSetpoint(shooterSetpoint);

    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

    // CAG: do we need encoders if we're just maintaining speeds? Probably not?
    // yes, it turns out:
    // https://github.com/PeddieRobotics/2022Gullinkambi/blob/dev/2022G/src/main/java/frc/robot/subsystems/Flywheel.java
    // TODO: understand and set up the feedforward controller
    // set up the PID controller
    // What are the methods to set it?

  }

  public void stopShooterMotor() {
    changeShooterSetpoint(0);
  }

  public void changeShooterSetpoint(double rpm) {
    shooterSetpoint = rpm;
    c_pid.setReference(shooterSetpoint, CANSparkMax.ControlType.kVelocity);
  }

  public void runShooterMotorFast() {
    // run the motor that changes the angle of the arm
    changeShooterSetpoint(ShooterConstants.kShooterFastSpeed);
  }

  public void runShooterMotorSlow() {
    // run the motor that changes the angle of the arm
    changeShooterSetpoint(ShooterConstants.kShooterSlowSpeed);
  }

  /*
   * public double getShooterMotorSpeed() {
   * return shooterMotor.get();
   * }
   */
  
  // CAG: AmIAtRPM-  who decides  what the  threshold is?

  @Override
  public void periodic() {
    if (ShooterConstants.AmTuningPID) {
      getShuffleBoard();
    }
    setShuffleBoard();
  }

  private void getShuffleBoard() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double setPoint = SmartDashboard.getNumber("Target", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if (p != c_pid.getP()) {
      c_pid.setP(p);
    }
    if (i != c_pid.getI()) {
      c_pid.setI(i);
    }
    if (d != c_pid.getD()) {
      c_pid.setD(d);
    }
    if (ff != c_pid.getFF()) {
      c_pid.setFF(ff);
    }
    if ((max != c_pid.getOutputMax()) || (min != c_pid.getOutputMin())) {
      c_pid.setOutputRange(min, max);
    }

    if (setPoint != shooterSetpoint) {
      changeShooterSetpoint(shooterSetpoint);
    }
  }

  private void setShuffleBoard() {
    SmartDashboard.putBoolean("Tuning ShooterPID", ShooterConstants.AmTuningPID);
    SmartDashboard.putNumber("SetPoint", shooterSetpoint);
    SmartDashboard.putNumber("Current Shooter RPM", -999);
    // SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }
}
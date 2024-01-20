// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ClawConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Claw extends SubsystemBase {
  private final CANSparkMax m_leftRollerMotor;
  private final CANSparkMax m_rightRollerMotor;
  //private final DoubleSolenoid m_leftPiston;
  private final DoubleSolenoid m_rightPiston;
  //private final DigitalInput m_clawBeamBreak;
  //private final ColorSensorV3 m_colorsensor;



  /** Creates a new Claw. */
  public Claw() {
    //m_clawBeamBreak = new DigitalInput(kBeamBreakPort);
    m_leftRollerMotor = new CANSparkMax(kLeftRollerMotorPort,MotorType.kBrushless);
    m_rightRollerMotor = new CANSparkMax(kRightRollerMotorPort,MotorType.kBrushless);
    //m_leftPiston = new DoubleSolenoid(kPneumaticsHubCanId,PneumaticsModuleType.REVPH, kLeftPistonExtendChannel, kLeftPistonRetractChannel);
    m_rightPiston = new DoubleSolenoid(kPneumaticsHubCanId,PneumaticsModuleType.REVPH, kRightPistonExtendChannel, kRightPistonRetractChannel);
m_rightPiston.set(kRightPistonRetractValue);
 // m_colorsensor = new ColorSensorV3(kColorSensorPort);

  }
  //initiates the roller motors to pick up game piece
public void runRollerMotors(double kRollerMotorspeed) {
  m_leftRollerMotor.set(kRollerMotorspeed);
  m_rightRollerMotor.set(kRollerMotorspeed);
}



//measures the motor current while the motors are running, once the current has spiked certain value (kCurrent) then we know we have a game piece.

//DURING TESTING, use shuffleboard to grab the current value for when the game piece is fully clasped.
public boolean hasPieceCube() {
  return !(m_rightRollerMotor.getOutputCurrent() > kCubeCurrent);
}


public boolean hasPieceCone() {
  return !(m_rightRollerMotor.getOutputCurrent() > kConeCurrent);
}

 //stops both motors
 public void stopRollerMotorsCone(double kRollerMotorConeStopSpeed) {
  if (hasPieceCone()== true) {
    m_leftRollerMotor.set(kRollerMotorConeStopSpeed);
    m_rightRollerMotor.set(kRollerMotorConeStopSpeed);
  } 


}

public void stopRollerMotorsCube(double kRollerMotorCubeStopSpeed) {
  if (hasPieceCube()== true) {
    m_leftRollerMotor.set(kRollerMotorCubeStopSpeed);
    m_rightRollerMotor.set(kRollerMotorCubeStopSpeed);
  } 


}


public void haltMotors(double kRollerMotorStopSpeed) {

    m_leftRollerMotor.set(kRollerMotorStopSpeed);
    m_rightRollerMotor.set(kRollerMotorStopSpeed);
}

//reverses the roller motors to eject game piece
public void ejectCubeRollerMotors(double kCubeShootSpeed) {
  m_leftRollerMotor.set(kCubeShootSpeed);
  m_rightRollerMotor.set(kCubeShootSpeed);
}

public void ejectConeRollerMotors(double kConeShootSpeed) {
  m_leftRollerMotor.set(kConeShootSpeed);
  m_rightRollerMotor.set(kConeShootSpeed);
}


public void extendRightPiston() {
//m_rightPiston.set(kRightPistonExtendValue);
m_rightPiston.set(kRightPistonExtendValue);
}

public void retractRightPiston() {
  m_rightPiston.set(kRightPistonRetractValue);

  

}

/*public void extendLeftPiston () {
  m_leftPiston.set(kLeftPistonExtendValue);
  }*/

  /*public void retractLeftPiston() {
    m_leftPiston.set(kLeftPistonRetractValue);
  }*/


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Have Cube", hasPieceCube());
    SmartDashboard.putBoolean("Have Cone", hasPieceCone());
    SmartDashboard.putNumber("Roller Current Output", m_rightRollerMotor.getOutputCurrent());
    // This method will be called once per scheduler run
  }
  
}
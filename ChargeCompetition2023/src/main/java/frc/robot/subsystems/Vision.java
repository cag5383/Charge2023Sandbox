// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
  
  private final NetworkTable m_limelightTable;
  private final NetworkTableEntry m_targetDetected;

  private double m_xOffset; // horizontal offset from crosshair to target (-27 degrees to 27 degrees)
  private double m_yOffset; // vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees)
  private double m_percentArea; // target area
  private double m_targetValue; // whether the limelight has an valid targets (0 or 1)

  private double[] m_targetPose; // Array of {x,y,z,rx,ry,rz} that will contain the position of the target (point of interest) relative to the camera📷
  private double[] defaultDoubleArray; // This will be a double array of length 6 that will be returned by default if the limelight doesn't detect anything 
  
  /** Creates a new Vision. */
  public Vision() {

    // Instantiate the network table
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Initialize the network table entries for target distance and target detected
    m_targetDetected = m_limelightTable.getEntry("Target Detected");

    // Instantiate the default double array
    defaultDoubleArray = new double[6];
    
    turnOnLED();
  }

  // Returns the horizontal offset to the target 
  public double getXOffset(){
    return m_xOffset;
  }

  // Returns the vertical offset to the target 
  public double getYOffset(){
    return m_yOffset;
  }

  // Returns the percent of the limelight that sees the area
  public double getpercentArea(){
    return m_percentArea;
  }

  // Returns target value (0 if has target 1 if not)
  public double getTargetValue(){
    return m_targetValue;
  }

  // Returns target position in camera space (z out from the front of the camera, x is to the right, y is down)
  public double[] getTargetPose(){
    return m_targetPose;
  }
  
  // Returns a boolean for if a target is currently detected
  public boolean isTargetDetected(){
    return (m_targetValue > 0.0);
  }

  //Returns angle in between direction robot is facing and point of interest
  public double getAprilTagAngle(){
    double aprilTagAngle;

    aprilTagAngle = Math.atan(m_targetPose[0]/m_targetPose[2]);
    aprilTagAngle = Math.toDegrees(aprilTagAngle);

    return aprilTagAngle;
  }

  public double getAprilTagDistance(){ //Grabs the distance assuming pointing directly at it
    return m_targetPose[2];
  }

  
  public void turnOnLED(){
    m_limelightTable.getEntry("LED Mode").setNumber(3);
  }

  public void turnOffLED(){
    m_limelightTable.getEntry("LED Mode").setNumber(1);
  }


  // Updates all the values to what the limelight is currently seeing
  public void updateLimeLight(){
    m_xOffset = m_limelightTable.getEntry("tx").getDouble(0.0);
    m_yOffset = m_limelightTable.getEntry("ty").getDouble(0.0);
    m_percentArea = m_limelightTable.getEntry("ta").getDouble(0.0);
    m_targetValue = m_limelightTable.getEntry("tv").getDouble(0.0);

    m_targetPose = m_limelightTable.getEntry("t6t_cs").getDoubleArray(defaultDoubleArray);

    m_targetDetected.setBoolean(isTargetDetected());
  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLimeLight();
    SmartDashboard.putNumber("tv", getTargetValue());
    SmartDashboard.putNumber("tx", getXOffset() );
    SmartDashboard.putNumber("ty", getYOffset());
    SmartDashboard.putNumber("ta", getpercentArea() );
  }
}

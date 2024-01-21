// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subsystemTemplates;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterTemplate extends SubsystemBase {

  // Definition - what hardware/sensors does the shooter need to interact with?
  // how many motors? Encoders? etc.

  // There's going to have to be a controller to maintain the shooter RPM
  // We're going to have to keep track of at least the set point of the
  // controller.

  public ShooterTemplate() {

    // General references for setting up a flywheel shooter subsystem:
    // https://github.com/PeddieRobotics/2022Gullinkambi/blob/dev/2022G/src/main/java/frc/robot/subsystems/Flywheel.java
    // https://github.com/2729StormRobotics/Charge2022/blob/Dev/ChargeRobotics2022/src/main/java/frc/robot/subsystems/Shooter.java

    // This is the shooter constructor - what are things I need to do on
    // initialization to be functional?

    // Configure the primary shooter motor
    // Configure the follower shooter motor
    // For both, make sure to restore factory defaults and set the idle mode.

    // For the primary shooter motor, create a PID controller - see
    // CANSparkMax.getPIDController() documentation.
    // Set the P, I, D, and FF values for the controller - preferably referencing
    // values from constants. See SparkPIDController.setP(), etc.
    // Set the minimum and maximum controller outputs. See
    // SparkPIDController.setOutputRange().
    // Initialize the target/setpoint to zero. See
    // SparkPIDController.setReference().
    //What should the controlType be? What are
    // the units of our measurement, target/set point, and minimum/maximum output?
    // Should we have any other safety concerns or hardware concerns we should protect against in software besides min/max output?

    // That's the end of the constructor - now we need to determine what subroutines
    // are required.

    // What basic methods does our shooter need to be able to function?
    // How do we stop the motor?
    // How should we change the target speed / set point?
    // Do other subsystems need to know what the current actual speed or target
    // speed are?
    // How will another subsystem know if the speed is close enough to the target to
    // shoot?
    // If there are specific combinations of commands that are going to show up in
    // many places, how can we consolidate them?

    // What more complex methods does out shooter need to be able to do more complex
    // tasks?
    // What does the rest of the robot need to know about how the shooter works?
    // Do we have a small-ish number of fixed speeds that the rest of the subsystems
    // can use? Or allow speeds to be determined more dynamically? What makes that
    // determination? How?

    // Tuning the controller is going to be the most difficult part of this
    // exercise.
    // One thing that will be easier than last year, is that the robot doesn't need
    // to drive to test the PID controller like it did with the charge station.
    // If we could change the PID values and the setpoint dynamically, we could test
    // very quickly how the system behaves when we try to ramp up.
    // The goal here is to use shuffleboard to change the controller dynamics
    // without having to rebuild/redeploy the robot code - this should make it
    // easier to test more quickly.

    // Take a look at the example below and play around with the Combined
    // Feedforward and Feedback Control example to get an idea of what some of these
    // controller parameters do and which ones we might need.
    // REFERENCE:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html

    // What values do we want to be able to change from shuffleboard in order to be
    // able to effectively test?
    // How do we know if they are different from what we currently have?
    // How do we check if there are changes and make the updates in the background
    // (i.e. periodically)
    // How do we actually update the parameters?
    // REFERENCE:
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

    // How do we review/plot the results?
    // Which controller parameters (P,I,D,FF) are we likely to need?
    // What are we looking for when tuning those particular controller parameters?
    // What should the methodology or order be?
  }
}

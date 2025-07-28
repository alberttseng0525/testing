// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class intake {
    public static final int MOTOR_ID = 54; // Intake motor ID
    public static final double intake = 0.45; // Intake speed
    public static final double carry = 0.2; // Carry speed
    public static final double L123 = -0.2; // Low speed for L1, L2, L3
    public static final double L4 = -0.8; // Low speed for L4
    public static final double Default = 0.0; // Default speed
  }

  public static class joint {
    public static final int MOTOR_ID = 53; // Joint motor ID
    public static final int CANCODER_ID = 25; // Joint CANCoder ID

    public static final double CANCORDER_GEAR_RATIO = 0.1; // Tolerance for joint position in degrees

    public static final double KS = 0.02; // Feedforward gain
    public static final double KV = 0.1; // Velocity gain
    public static final double KA = 0.1; // Acceleration gain
    public static final double KG = 0.01; // Gravity compensation gain

    public static final double KP = 4.0; // Proportional gain  
    public static final double KI = 0.0; // Integral gain
    public static final double KD = 0.2; // Derivative gain
    //public static final double KF = 0.1; // Feedforward gain

    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine; // Gravity compensation type

    public static final double CRUISE_VELOCITY = 0.2; // Motion magic cruise velocity
    public static final double ACCELERATION = 0.6; // Motion magic acceleration
    public static final double JERK = 0.0; // Tolerance for joint position in degrees

    public static final double STATOR_CURRENT_LIMIT = 30.0; // Stator current limit in Amperes
    public static final double SUPPLY_CURRENT_LIMIT = 30.0; // Supply current limit in Amperes
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0; // Supply current lower time in seconds

    public static final double ABSOLUTE_SENSOR_DISCONTINUITY_POINT = 0.5; // Absolute sensor discontinuity point in degrees
    public static final double MAGNET_OFFSET = -0.368164; // Absolute sensor offset in degrees
   
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class FuelConstants {
    // Motor controller CAN IDs for Fuel mechanisms
    public static final int INTAKE_MOTOR_ID = 16;
    public static final int FEEDER_MOTOR_ID = 15;
    public static final int LAUNCHER_MOTOR_ID_1 = 14;    
    public static final int LAUNCHER_MOTOR_ID_2 = 17;


    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = 8;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 10;
    public static final double SPIN_UP_FEEDER_VOLTAGE = 0;
  
    // RPMs for shooter
    public static final double DEFAULT_LAUNCH_RPM = 3500;
    public static final double AT_HUB_LAUNCH_RPM = 2500;

    // When shooter is at + or - this RPM, it is considered at its setpoint
    public static final double LAUNCH_TOLERANCE = .5;

    // PID controls for rotating bot to shoot
    public static final double SHOOT_AIM_KP = .01;
    public static final double SHOOT_AIM_KI = 0;
    public static final double SHOOT_AIM_KD = 0;

    // This number multiplies by the distance between the bot and the hub, and is added to AT_HUB_RPM
    public static final double LIMELIGHT_RPM_KP = .01;

    public static final boolean USE_SHOOTER_LIMELIGHT = true;

    // Coordinates are in meters from the right corner on the blue alliance side
    public static final double HUB_X_COORD = 4.625594;
    public static final double HUB_Y_COORD = 4.03352;
    public static final double FULL_FIELD_X = 16.5409;
    public static final double FULL_FIELD_Y = 8.0693;
    // how many seconds does it take to launch eight fuel (used for auto)
    public static final double TIME_TO_LAUNCH_8 = 3;
    public static final double TIME_TO_LAUNCH_ALL = 5;

  }

  public static final class ClimbConstants {
    // CAN ID for climb kraken
    public static final int CLIMB_MOTOR_ID = 18;
    // Current for the motor for the Climber
    public static final int MAX_CLIMB_CURRENT = 60;
    // Time for motor to rotate in one direction, in seconds
    public static final long CLIMB_CYCLE_TIME = 3;
    // PID constants for lining up the robot with the tower
    public static final double ALIGN_XY_KP = 0;
    public static final double ROTATION_KP = 0;
    // Whether to use auto-alignment
    public static final boolean USE_CLIMB_ALIGNMENT = false;

  }

  public static final class DriveConstants {
    // This number is multiplied by joystick input and max speed to get target velocities
    public static final double SPEED_SCALAR = .7;
    // Same thing, but only applied while the robot is shooting and pointing itself to the hub
    public static final double SPEED_SCALAR_WHILE_TARGETING = .5;
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class ControllerDriverConstants { // Logitech Gamepad F310 in X Mode
    public static final int JOYSTICK_PORT = 0;

    public static final int AXIS_LEFT_Y = 1;
    public static final int AXIS_LEFT_X = 0;
    public static final int AXIS_RIGHT_Y = 5;
    public static final int AXIS_RIGHT_X = 4;

    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_X = 3;

    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;

    public static final int BUTTON_BUMPER_RIGHT = 6;
    public static final int BUTTON_BUMPER_LEFT = 5;

    public static final double DEADBAND_AXIS_LEFT_Y = 0.05;
    public static final double DEADBAND_AXIS_LEFT_X = 0.05;
    public static final double DEADBAND_AXIS_RIGHT_Y = 0.05;
    public static final double DEADBAND_AXIS_RIGHT_X = 0.05;
  }

  public static class ControllerOperatorConstants { // Logitech Gamepad F310 in D Mode
    public static final int JOYSTICK_PORT = 1;

    public static final int AXIS_LEFT_Y = 1;
    public static final int AXIS_LEFT_X = 0;
    public static final int AXIS_RIGHT_Y = 3;
    public static final int AXIS_RIGHT_X = 2;

    public static final int BUTTON_A = 2;
    public static final int BUTTON_B = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_X = 1;

    public static final int BUTTON_BACK = 9;
    public static final int BUTTON_START = 10;

    public static final int BUTTON_BUMPER_TOP_RIGHT = 6;
    public static final int BUTTON_BUMPER_TOP_LEFT = 5;
    public static final int BUTTON_BUMPER_BOTTOM_RIGHT = 8;
    public static final int BUTTON_BUMPER_BPTTOM_LEFT = 7;

    public static final double DEADBAND_AXIS_LEFT_Y = 0.05;
    public static final double DEADBAND_AXIS_LEFT_X = 0.05;
    public static final double DEADBAND_AXIS_RIGHT_Y = 0.05;
    public static final double DEADBAND_AXIS_RIGHT_X = 0.05;
  }

  public static class ElevatorConstants {
    public static final int MOTOR_LEFT_CANID = 20;
    public static final MotorType MOTOR_LEFT_TYPE = MotorType.kBrushless;
    public static final boolean MOTOR_LEFT_IS_INVERTED = true;

    public static final int MOTOR_RIGHT_CANID = 21;
    public static final MotorType MOTOR_RIGHT_TYPE = MotorType.kBrushless;
    public static final boolean MOTOR_RIGHT_IS_INVERTED = true;

    public static final double SLOT_ZERO_P = 0.5;
    public static final double SLOT_ZERO_I = 0.0;
    public static final double SLOT_ZERO_D = 0.0;
    public static final double SLOT_ZERO_F = 0.0;

    public static final double HEIGHT_CORAL_LEVEL_FOUR = -52.2; // Estimated based on manual lift.
    public static final double HEIGHT_CORAL_LEVEL_THREE = -28.00; // Almost worked at 25. Need a hair down.
    public static final double HEIGHT_CORAL_LEVEL_TWO = -12.00; // Almost worked at 10. Need a hair down.
    public static final double HEIGHT_CORAL_LEVEL_ONE = -5; // UNKOWN
    public static final double HEIGHT_CORAL_INTAKE = -0.25; // UNKOWN
    public static final double HEIGHT_STOWED = -0.5; // UNKOWN
    public static final double HEIGHT_NO_POWER = 0.00;
  }

  public static class CoralConstants {

  }

  public static class FlapConstants {

  }

}

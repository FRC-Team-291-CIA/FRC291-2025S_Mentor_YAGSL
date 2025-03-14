// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose.
 * All constants should be declared globally (i.e. public static). Do not put
 * anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Robot mass in kilograms (subtracting 20.3 lbs for battery and bumpers)
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound

  // Chassis mass representation using the Matter class
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

  // Control loop cycle time (20ms base loop time + 110ms Spark MAX velocity lag)
  public static final double LOOP_TIME = 0.13; // seconds

  // Maximum robot speed in meters per second
  public static final double MAX_SPEED = Units.feetToMeters(14.50);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // Constants for autonomous mode (commented out for now)
  // public static final class AutonConstants
  // {
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {
    // Time in seconds that the wheels will remain locked (brakes applied) when the
    // robot is disabled
    public static final double WHEEL_LOCK_TIME = 10.00; // seconds
  }

  public static class OperatorConstants {
    // Deadband settings for joystick input to prevent unintended small movements
    public static final double DEADBAND = 0.10;
    public static final double LEFT_Y_DEADBAND = 0.10;
    public static final double RIGHT_X_DEADBAND = 0.10;

    // Turn sensitivity multiplier
    public static final double TURN_CONSTANT = 6.00;
  }

  public static class ControllerDriverConstants { // Logitech Gamepad F310 in X Mode

    // Controller USB port
    public static final int JOYSTICK_PORT = 0;

    // Joystick axis mappings
    public static final int AXIS_LEFT_Y = 1;
    public static final int AXIS_LEFT_X = 0;
    public static final int AXIS_RIGHT_Y = 5;
    public static final int AXIS_RIGHT_X = 4;

    // Button mappings
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_X = 3;

    public static final int BUTTON_JOYSTICK_LEFT = 9;
    public static final int BUTTON_JOYSTICK_RIGHT = 10;

    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;

    public static final int BUTTON_BUMPER_RIGHT = 6;
    public static final int BUTTON_BUMPER_LEFT = 5;

    // Deadband settings for joystick input
    public static final double DEADBAND = 0.05;
    public static final double DEADBAND_AXIS_LEFT_Y = 0.05;
    public static final double DEADBAND_AXIS_LEFT_X = 0.05;
    public static final double DEADBAND_AXIS_RIGHT_Y = 0.05;
    public static final double DEADBAND_AXIS_RIGHT_X = 0.05;
  }

  public static class ControllerOperatorConstants { // Logitech Gamepad F310 in D Mode

    // Controller USB port
    public static final int JOYSTICK_PORT = 1;

    // Joystick axis mappings
    public static final int AXIS_LEFT_Y = 1;
    public static final int AXIS_LEFT_X = 0;
    public static final int AXIS_RIGHT_Y = 3;
    public static final int AXIS_RIGHT_X = 2;

    // Button mappings
    public static final int BUTTON_A = 2;
    public static final int BUTTON_B = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_X = 1;

    public static final int BUTTON_BACK = 9;
    public static final int BUTTON_START = 10;

    public static final int BUTTON_BUMPER_TOP_RIGHT = 6;
    public static final int BUTTON_BUMPER_TOP_LEFT = 5;
    public static final int BUTTON_BUMPER_BOTTOM_RIGHT = 8;
    public static final int BUTTON_BUMPER_BOTTOM_LEFT = 7;

    // Deadband settings for joystick input
    public static final double DEADBAND_AXIS_LEFT_Y = 0.05;
    public static final double DEADBAND_AXIS_LEFT_X = 0.05;
    public static final double DEADBAND_AXIS_RIGHT_Y = 0.05;
    public static final double DEADBAND_AXIS_RIGHT_X = 0.05;
  }

  public static class ElevatorConstants {
    // Enables or disables anti-motor crash testing feature
    public static final boolean TEST_ANTI_MOTOR_CRASH_IS_ENABLED = false;

    // Percentage values for motor crash prevention thresholds
    public static final double ANTI_CRASH_PERCENT_FROM_TOP = 0.9;
    public static final double ANTI_CRASH_PERCENT_ABOVE_BOT = 1.1;

    // Motor configuration for elevator
    public static final int MOTOR_LEFT_CANID = 20;
    public static final MotorType MOTOR_LEFT_TYPE = MotorType.kBrushless;
    public static final boolean MOTOR_LEFT_IS_INVERTED = true;

    public static final int MOTOR_RIGHT_CANID = 21;
    public static final MotorType MOTOR_RIGHT_TYPE = MotorType.kBrushless;
    public static final boolean MOTOR_RIGHT_IS_INVERTED = true;

    // Conversion factor for motor revolutions per inch of elevator travel
    public static final double MOTOR_REVOLUTION_PER_INCH = (4335.00 / 4698.00);

    // Current limit for smart motor control
    public static final int MOTOR_SMART_CURRENT_LIMIT = 40;

    // PID slot 0 constants for motor control
    public static final double SLOT_ZERO_P = 0.10;
    public static final double SLOT_ZERO_I = 0.00;
    public static final double SLOT_ZERO_D = 0.00;
    public static final double SLOT_ZERO_FF = -0.00; // -0.60
    public static final ArbFFUnits SLOT_ZERO_FF_UNITS = ArbFFUnits.kVoltage;
    public static final double SLOT_ZERO_MAX_ACCELERATION = 2000.00;
    public static final double SLOT_ZERO_MAX_VELOCITY = 4000.00;

    // PID slot 1 constants for motor control
    public static final double SLOT_ONE_P = 0.3;
    public static final double SLOT_ONE_I = 0.00;
    public static final double SLOT_ONE_D = 0.00;
    public static final double SLOT_ONE_FF = 0.00;
    public static final ArbFFUnits SLOT_ONE_FF_UNITS = ArbFFUnits.kPercentOut;
    public static final double SLOT_ONE_MAX_ACCELERATION = 1000.00;
    public static final double SLOT_ONE_MAX_VELOCITY = 1000.00;

    // Predefined elevator height positions for different levels
    public static final double HEIGHT_CORAL_LEVEL_FOUR = -52.20; // Estimated based on manual lift.
    public static final double HEIGHT_CORAL_LEVEL_THREE = -28.00; // Slightly lower than 25 for best fit.
    public static final double HEIGHT_CORAL_LEVEL_TWO = -12.00; // Slightly lower than 10 for best fit.
    public static final double HEIGHT_CORAL_LEVEL_ONE = -5.00; // Unknown value.
    public static final double HEIGHT_CORAL_INTAKE = -0.25; // Unknown value.
    public static final double HEIGHT_PARK = 0.00;
    public static final double HEIGHT_KILL_POWER = Double.NaN;
    public static final double HEIGHT_DISABLED = Double.NaN;
  }

  public static class CoralConstants {
    // Motor configuration for coral mechanism
    public static final int MOTOR_LEFT_CANID = 22;
    public static final MotorType MOTOR_LEFT_TYPE = MotorType.kBrushed;
    public static final boolean MOTOR_LEFT_IS_INVERTED = false;

    public static final int MOTOR_RIGHT_CANID = 23;
    public static final MotorType MOTOR_RIGHT_TYPE = MotorType.kBrushed;
    public static final boolean MOTOR_RIGHT_IS_INVERTED = true;

    // Sensor configuration for coral intake detection
    public static final int INTAKE_SENSOR_DIOPORT = 0;
    public static final boolean INTAKE_SENSOR_IS_INVERTED = false;

    // Manual control speeds for intake system
    public static final double SPEED_MANUAL_FORWARD_SLOW = 0.2;
    public static final double SPEED_MANUAL_FORWARD_FAST = 0.4;
    public static final double SPEED_MANUAL_REVERSE_SLOW = -0.2;
    public static final double SPEED_MANUAL_REVERSE_FAST = -0.4;
  }

  public static class FlapConstants {
    public static final boolean TEST_STATE_BASED = false;

    // Motor configuration for flap mechanism
    public static final int MOTOR_CANID = 24;
    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
    public static final boolean MOTOR_IS_INVERTED = false;

    // Current limit for smart motor control
    public static final int MOTOR_SMART_CURRENT_LIMIT = 40;

    // Conversion factor for motor revolutions per inch of flap travel
    public static final double MOTOR_REVOLUTION_PER_INCH = 1;

    // PID slot 0 constants for motor control
    public static final double SLOT_ZERO_P = 0.50;
    public static final double SLOT_ZERO_I = 0.00;
    public static final double SLOT_ZERO_D = 0.00;
    public static final double SLOT_ZERO_FF = 0.00;
    public static final ArbFFUnits SLOT_ZERO_FF_UNITS = ArbFFUnits.kPercentOut;
    public static final double SLOT_ZERO_MAX_ACCELERATION = 1000.00;
    public static final double SLOT_ZERO_MAX_VELOCITY = 1000.00;

    // Predefined angle positions for the flap
    public static final double ANGLE_UP = 90;
    public static final double ANGLE_DOWN = 0;
    public static final double ANGLE_DISBALED = Double.NaN;

    public static final double UP_SPEED = 0.5;
    public static final double DOWN_SPEED = -0.5;
  }

  public static class ClimberConstants {
    public static final int MOTOR_CANID = 25;
    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
    public static final boolean MOTOR_IS_INVERTED = false;
    public static final IdleMode MOTOR_MODE = IdleMode.kBrake;
    public static final int MOTOR_SMART_CURRENT_LIMIT = 40;

    public static final double SPEED_IN = 0.5;
    public static final double SPEED_OUT = -0.5;
  }

  public static class CIAAutoConstants {
    // Speeds for automatic coral handling during autonomous mode
    public static final double SPEED_AUTO_CORAL_BEFORE_ENTER = 0.4;
    public static final double SPEED_AUTO_CORAL_AFTER_ENTER = 0.1;
  }

}

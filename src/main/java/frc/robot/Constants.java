// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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

  public static final class driverConstants { // Logitech Gamepad F310 in X Mode
    public static final int kJoystickPort = 0;

    public static final int kAxisLeftY = 1; // Left Y Axis
    public static final int kAxisLeftX = 0; // Left X Axis
    public static final int kAxisRightY = 5; // Left Y Axis
    public static final int kAxisRightX = 4; // Right X Axis

    public static final int kButtonA = 1;
    public static final int kButtonB = 2;
    public static final int kButtonY = 4;
    public static final int kButtonX = 3;

    public static final int kButtonBack = 7;
    public static final int kButtonStart = 8;

    public static final int kButtonRightBumper = 6;
    public static final int kButtonLeftBumper = 5;

    public static final double kDeadbandAxisY = 0.05;
    public static final double kDeadbandAxisX = 0.05;
    public static final double kDeadBandAxisRotation = 0.05;

    // USED BY YAGSL
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class operatorConstants { // Logitech Gamepad F310 in X Mode
    public static final int kJoystickPort = 1;

    public static final int kAxisLeftY = 1; // Left Y Axis
    public static final int kAxisLeftX = 0; // Left X Axis
    public static final int kAxisRightY = 5; // Left Y Axis
    public static final int kAxisRightX = 4; // Right X Axis

    public static final int kButtonA = 1;
    public static final int kButtonB = 2;
    public static final int kButtonY = 4;
    public static final int kButtonX = 3;

    public static final int kButtonBack = 7;
    public static final int kButtonStart = 8;

    public static final int kButtonRightBumper = 6;
    public static final int kButtonLeftBumper = 5;

    public static final double kDeadbandAxisY = 0.05;
    public static final double kDeadbandAxisX = 0.05;
    public static final double kDeadBandAxisRotation = 0.05;
  }

  public static class ElevatorConstants {
    public static final int motorLeftCANID = 20;
    public static final int motorRightCANID = 21;

    public static final MotorType motorLeftType = MotorType.kBrushless;
    public static final MotorType motorRightType = MotorType.kBrushless;

    public static final boolean motorLeftInvert = true;
    public static final boolean motorRightInvert = true;

    public static final double HEIGHT_CORAL_LEVEL_FOUR = -49.00; //Estimated based on manual lift. 
    public static final double HEIGHT_CORAL_LEVEL_THREE = -24.00; //Almost worked at 25. Need a hair down.
    public static final double HEIGHT_CORAL_LEVEL_TWO = -8.00; //Almost worked at 10. Need a hair down.
    public static final double HEIGHT_CORAL_LEVEL_ONE = -5; //UNKOWN
    public static final double HEIGHT_CORAL_INTAKE = -4; //UNKOWN
    public static final double HEIGHT_STOWED = -2; //UNKOWN
    public static final double HEIGHT_PARK = 0.00;
    public static final double HEIGHT_NO_POWER = 0.00;

    public static final double MOVING_UP_P = 0.3;
    public static final double MOVING_UP_I = 0.0;
    public static final double MOVING_UP_D = 0.0;

    public static final double HOLDING_P = 0.2;
    public static final double HOLDING_I = 0.0;
    public static final double HOLDING_D = 0.0;

    public static final double MOVING_DOWN_P = 0.075;
    public static final double MOVING_DOWN_I = 0.0;
    public static final double MOVING_DOWN_D = 0.0;
  }

  public static class CoralConstants {
    public static final int motorLeftCANID = 22;
    public static final int motorRightCANID = 23;

    public static final MotorType motorLeftType = MotorType.kBrushed;
    public static final MotorType motorRightType = MotorType.kBrushed;

    public static final boolean motorLeftInvert = false;
    public static final boolean motorRightInvert = true;

    public static final double inSpeed = 0.35;
    public static final double outSpeed = 0.6;
  }

}

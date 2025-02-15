// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.NamedCommands;

import swervelib.SwerveInputStream;

import frc.robot.Constants.driverConstants;
import frc.robot.Constants.operatorConstants;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

        public final CommandJoystick driverJoystick = new CommandJoystick(driverConstants.kJoystickPort);
        public final CommandJoystick operatorJoystick = new CommandJoystick(operatorConstants.kJoystickPort);

        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve"));

        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

        private final CoralSubsystem coralSubsystem = new CoralSubsystem();

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverJoystick.getHID().getRawAxis(driverConstants.kAxisLeftY) * -1,
                        () -> driverJoystick.getHID().getRawAxis(driverConstants.kAxisLeftX) * -1)
                        .withControllerRotationAxis(
                                        () -> driverJoystick.getHID().getRawAxis(driverConstants.kAxisRightX) * -1)
                        .deadband(
                                        driverConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(
                                        () -> driverJoystick.getHID().getRawAxis(driverConstants.kAxisRightX),
                                        () -> driverJoystick.getHID().getRawAxis(driverConstants.kAxisRightY))
                        .headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverJoystick.getHID().getRawAxis(driverConstants.kAxisLeftY),
                        () -> -driverJoystick.getHID().getRawAxis(driverConstants.kAxisLeftX))
                        .withControllerRotationAxis(() -> driverJoystick.getRawAxis(driverConstants.kAxisRightX))
                        .deadband(
                                        driverConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverJoystick.getRawAxis(
                                                        2) *
                                                        Math.PI)
                                        *
                                        (Math.PI *
                                                        2),
                                        () -> Math.cos(
                                                        driverJoystick.getRawAxis(
                                                                        2) *
                                                                        Math.PI)
                                                        *
                                                        (Math.PI *
                                                                        2))
                        .headingWhile(true);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.butt on.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {
                SmartDashboard.putData(elevatorSubsystem);

                Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
                Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngle);
                Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);
                Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngleKeyboard);

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }

                if (Robot.isSimulation()) {
                        driverJoystick.button(driverConstants.kButtonStart)
                                        .onTrue(Commands.runOnce(() -> drivebase
                                                        .resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
                        driverJoystick.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
                }

                driverJoystick.button(driverConstants.kButtonA).onTrue((Commands.runOnce(drivebase::zeroGyro)));

                driverJoystick.button(driverConstants.kButtonX).whileTrue(Commands.none());

                driverJoystick.button(driverConstants.kButtonB).whileTrue(
                                drivebase.driveToPose(
                                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));

                driverJoystick.button(driverConstants.kButtonStart).whileTrue(Commands.none());

                driverJoystick.button(driverConstants.kButtonBack).whileTrue(coralSubsystem.cIn());

                driverJoystick.button(driverConstants.kButtonLeftBumper)
                                .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

                driverJoystick.button(driverConstants.kButtonRightBumper).whileTrue(coralSubsystem.cOut());

                operatorJoystick.button(operatorConstants.kButtonA)
                                .onTrue(Commands.runOnce(
                                                () -> elevatorSubsystem.setWantedState(ElevatorState.CORAL_INTAKE)));

                operatorJoystick.button(operatorConstants.kButtonB)
                                .onTrue(Commands.runOnce(
                                                () -> elevatorSubsystem.setWantedState(ElevatorState.CORAL_LEVEL_TWO)));

                operatorJoystick.button(operatorConstants.kButtonY)
                                .onTrue(Commands.runOnce(() -> elevatorSubsystem
                                                .setWantedState(ElevatorState.CORAL_LEVEL_THREE)));

                operatorJoystick.button(operatorConstants.kButtonX)
                                .onTrue(Commands.runOnce(() -> elevatorSubsystem
                                                .setWantedState(ElevatorState.CORAL_LEVEL_FOUR)));

                operatorJoystick.button(operatorConstants.kButtonBack)
                                .onTrue(Commands.runOnce(
                                                () -> elevatorSubsystem.setWantedState(ElevatorState.NO_POWER)));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return drivebase.getAutonomousCommand("New Auto");
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}

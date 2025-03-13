// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.ControllerDriverConstants;
import frc.robot.Constants.ControllerOperatorConstants;
import frc.robot.Constants.FlapConstants;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.flap.FlapSubsystem;
import frc.robot.subsystems.flap.FlapSubsystem.FlapState;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;

import frc.robot.commands.IntakeCoralCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

        final CommandJoystick controllerDriver = new CommandJoystick(ControllerDriverConstants.JOYSTICK_PORT);
        final CommandJoystick controllerOperator = new CommandJoystick(ControllerOperatorConstants.JOYSTICK_PORT);

        // The robot's subsystems and commands are defined here
        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve"));

        private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
        private final FlapSubsystem m_flapSubsystem = new FlapSubsystem();
        private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
        private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> controllerDriver.getRawAxis(ControllerDriverConstants.AXIS_LEFT_Y) * -1,
                        () -> controllerDriver.getRawAxis(ControllerDriverConstants.AXIS_LEFT_X) * -1)
                        .withControllerRotationAxis(() -> controllerDriver.getRawAxis(
                                        ControllerDriverConstants.AXIS_RIGHT_X)
                                        * -1)
                        .deadband(
                                        ControllerDriverConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
                        () -> controllerDriver.getRawAxis(
                                        ControllerDriverConstants.AXIS_RIGHT_X),
                        () -> controllerDriver.getRawAxis(
                                        ControllerDriverConstants.AXIS_RIGHT_X))
                        .headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> controllerDriver.getRawAxis(
                                        ControllerDriverConstants.AXIS_LEFT_Y),
                        () -> controllerDriver.getRawAxis(
                                        ControllerDriverConstants.AXIS_LEFT_X))
                        .withControllerRotationAxis(() -> controllerDriver.getRawAxis(
                                        ControllerDriverConstants.AXIS_RIGHT_X))
                        .deadband(
                                        ControllerDriverConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        controllerDriver.getRawAxis(
                                                        ControllerDriverConstants.AXIS_RIGHT_X) *
                                                        Math.PI)
                                        *
                                        (Math.PI *
                                                        2),
                                        () -> Math.cos(
                                                        controllerDriver.getRawAxis(
                                                                        ControllerDriverConstants.AXIS_RIGHT_X) *
                                                                        Math.PI)
                                                        *
                                                        (Math.PI *
                                                                        2))
                        .headingWhile(true)
                        .translationHeadingOffset(true)
                        .translationHeadingOffset(Rotation2d.fromDegrees(
                                        0));

        SendableChooser<Command> m_chooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                m_chooser.setDefaultOption("DO NOTHING", Commands.none());
                m_chooser.addOption("Left Two Auto", drivebase.getAutonomousCommand("Left Two Auto"));

                SmartDashboard.putData(m_chooser);
                SmartDashboard.putData("CoralSubsystem", m_coralSubsystem);

                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
                NamedCommands.registerCommand("Coral Intake", new IntakeCoralCommand(m_coralSubsystem));
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
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {
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
                        Pose2d target = new Pose2d(new Translation2d(1, 4),
                                        Rotation2d.fromDegrees(90));
                        // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
                        driveDirectAngleKeyboard.driveToPose(() -> target,
                                        new ProfiledPIDController(5,
                                                        0,
                                                        0,
                                                        new Constraints(5, 2)),
                                        new ProfiledPIDController(5,
                                                        0,
                                                        0,
                                                        new Constraints(Units.degreesToRadians(360),
                                                                        Units.degreesToRadians(180))));
                        controllerDriver.button(ControllerDriverConstants.BUTTON_START)
                                        .onTrue(Commands.runOnce(() -> drivebase
                                                        .resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
                        controllerDriver.button(ControllerDriverConstants.BUTTON_A)
                                        .whileTrue(drivebase.sysIdDriveMotorCommand());
                        controllerDriver.button(
                                        ControllerDriverConstants.BUTTON_B)
                                        .whileTrue(Commands.runEnd(
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

                        // driverXbox.b().whileTrue(
                        // drivebase.driveToPose(
                        // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                        // );

                }
                if (DriverStation.isTest()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command
                                                                                         // above!

                        controllerDriver.button(
                                        ControllerDriverConstants.BUTTON_X)
                                        .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                        controllerDriver.button(
                                        ControllerDriverConstants.BUTTON_Y)
                                        .whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
                        controllerDriver.button(
                                        ControllerDriverConstants.BUTTON_START)
                                        .onTrue((Commands.runOnce(drivebase::zeroGyro)));
                        controllerDriver.button(
                                        ControllerDriverConstants.BUTTON_BACK)
                                        .whileTrue(drivebase.centerModulesCommand());
                        controllerDriver.button(
                                        ControllerDriverConstants.BUTTON_BUMPER_LEFT).onTrue(Commands.none());
                        controllerDriver.button(
                                        ControllerDriverConstants.BUTTON_BUMPER_RIGHT).onTrue(Commands.none());
                } else {
                        // ---------------------------- Driver Controller ---------------------------

                        controllerDriver.button(
                                        ControllerDriverConstants.BUTTON_A)
                                        .onTrue((Commands.runOnce(drivebase::zeroGyro)));

                        controllerDriver.button(
                                        ControllerDriverConstants.BUTTON_BUMPER_LEFT)
                                        .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

                        controllerDriver.button(ControllerDriverConstants.BUTTON_JOYSTICK_RIGHT).onTrue(Commands
                                        .runOnce(() -> m_elevatorSubsystem.setWantedState(ElevatorState.CORAL_INTAKE)));

                        controllerDriver.button(ControllerDriverConstants.BUTTON_BUMPER_LEFT)
                                        .whileTrue(Commands.run(m_climberSubsystem::out, m_climberSubsystem));

                        controllerDriver.button(ControllerDriverConstants.BUTTON_BUMPER_RIGHT)
                                        .whileTrue(Commands.run(m_climberSubsystem::in, m_climberSubsystem));

                        // ---------------------------- Operator Controller ---------------------------
                        // Elevator
                        controllerOperator.button(ControllerOperatorConstants.BUTTON_A).onTrue(Commands
                                        .runOnce(() -> m_elevatorSubsystem.setWantedState(ElevatorState.CORAL_INTAKE)));

                        controllerOperator.button(ControllerOperatorConstants.BUTTON_B).onTrue(Commands
                                        .runOnce(() -> m_elevatorSubsystem
                                                        .setWantedState(ElevatorState.CORAL_LEVEL_TWO)));

                        controllerOperator.button(ControllerOperatorConstants.BUTTON_Y).onTrue(Commands
                                        .runOnce(() -> m_elevatorSubsystem
                                                        .setWantedState(ElevatorState.CORAL_LEVEL_THREE)));

                        controllerOperator.button(ControllerOperatorConstants.BUTTON_X).onTrue(Commands
                                        .runOnce(() -> m_elevatorSubsystem
                                                        .setWantedState(ElevatorState.CORAL_LEVEL_FOUR)));

                        controllerOperator.button(ControllerOperatorConstants.BUTTON_START).onTrue(Commands
                                        .runOnce(() -> m_elevatorSubsystem
                                                        .setWantedState(ElevatorState.PARK)));

                        controllerOperator.button(ControllerOperatorConstants.BUTTON_BACK).onTrue(Commands
                                        .runOnce(() -> m_elevatorSubsystem
                                                        .setWantedState(ElevatorState.NO_POWER)));
                        // Flap

                        if (FlapConstants.TEST_STATE_BASED) {
                                controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_BOTTOM_LEFT)
                                                .onTrue(Commands
                                                                .runOnce(() -> m_flapSubsystem
                                                                                .setWantedState(FlapState.DOWN)));

                                controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_BOTTOM_RIGHT)
                                                .onTrue(Commands
                                                                .runOnce(() -> m_flapSubsystem
                                                                                .setWantedState(FlapState.UP)));

                        } else {
                                controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_BOTTOM_LEFT)
                                                .whileTrue(Commands
                                                                .run(() -> m_flapSubsystem
                                                                                .setWantedState(FlapState.DOWN)));

                                controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_BOTTOM_RIGHT)
                                                .whileTrue(Commands
                                                                .run(() -> m_flapSubsystem
                                                                                .setWantedState(FlapState.UP)));
                        }

                        // Coral
                        controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_TOP_LEFT).whileTrue(
                                        new IntakeCoralCommand(m_coralSubsystem));

                        controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_TOP_RIGHT)
                                        .whileTrue(m_coralSubsystem.MANUAL_FORWARD_FAST());

                        controllerOperator.pov(0).whileTrue(m_coralSubsystem.MANUAL_FORWARD_SLOW());
                        controllerOperator.pov(180)
                                        .whileTrue(m_coralSubsystem.MANUAL_REVERSE_SLOW());
                }

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return m_chooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}

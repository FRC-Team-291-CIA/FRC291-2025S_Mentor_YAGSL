package frc.robot.subsystems.elevator;

// Import necessary libraries and dependencies
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.Constants.ElevatorConstants;

// Define the ElevatorSubsystem class which extends SubsystemBase
public class ElevatorSubsystem extends SubsystemBase {

    // Declare motor controllers, configurations, encoders, and controllers
    private final SparkMax m_motorLeft, m_motorRight;
    private final SparkMaxConfig m_configLeft, m_configRight;
    private final RelativeEncoder m_encoderLeft, m_encoderRight;
    private final SparkClosedLoopController m_controllerLeft, m_controllerRight;

    private double m_elevatorHeight = -291.00; // Default height for debugging purposes

    // Enum to represent different elevator states
    public enum ElevatorState {
        CORAL_LEVEL_FOUR(ElevatorConstants.HEIGHT_CORAL_LEVEL_FOUR, "CORAL LEVEL 4"),
        CORAL_LEVEL_THREE(ElevatorConstants.HEIGHT_CORAL_LEVEL_THREE, "CORAL LEVEL 3"),
        CORAL_LEVEL_TWO(ElevatorConstants.HEIGHT_CORAL_LEVEL_TWO, "CORAL LEVEL 2"),
        CORAL_LEVEL_ONE(ElevatorConstants.HEIGHT_CORAL_LEVEL_ONE, "CORAL LEVEL 1"),
        CORAL_INTAKE(ElevatorConstants.HEIGHT_CORAL_INTAKE, "CORAL INTAKE"),
        PARK(ElevatorConstants.HEIGHT_PARK, "PARK"),
        NO_POWER(ElevatorConstants.HEIGHT_KILL_POWER, "KILL POWER"),
        DISABLED(ElevatorConstants.HEIGHT_DISABLED, "DISABLED");

        private final double height;
        private final String name;

        // Constructor for ElevatorState enum
        ElevatorState(double height, String name) {
            this.height = height;
            this.name = name;
        }

        // Getter method for height
        public double getHeight() {
            return height;
        }

        // Getter method for name
        public String getName() {
            return name;
        }
    }

    // Enum for different motor control states
    private enum MotorState {
        NORMAL(ClosedLoopSlot.kSlot0, ElevatorConstants.SLOT_ZERO_FF_UNITS, ElevatorConstants.SLOT_ZERO_FF, "NORMAL"),
        ANTI_CRASH(ClosedLoopSlot.kSlot1, ElevatorConstants.SLOT_ONE_FF_UNITS, ElevatorConstants.SLOT_ONE_FF,
                "ANTI CRASH");

        private final ClosedLoopSlot closedLoopSlot;
        private final ArbFFUnits arbFFUnits;
        private final double ff;
        private final String name;

        // Constructor for MotorState enum
        MotorState(ClosedLoopSlot closedLoopSlot, ArbFFUnits arbFFUnits, double ff, String name) {
            this.closedLoopSlot = closedLoopSlot;
            this.arbFFUnits = arbFFUnits;
            this.ff = ff;
            this.name = name;
        }

        // Getter methods for MotorState properties
        public ClosedLoopSlot getClosedLoopSlot() {
            return closedLoopSlot;
        }

        public ArbFFUnits getArbFFUnits() {
            return arbFFUnits;
        }

        public double getFF() {
            return ff;
        }

        public String getName() {
            return name;
        }
    }

    // Declare and initialize current elevator and motor states
    public ElevatorState m_currentElevatorState = ElevatorState.DISABLED;
    private MotorState m_currentMotorState = MotorState.NORMAL;

    // Constructor for ElevatorSubsystem
    public ElevatorSubsystem() {
        // Initialize left and right elevator motors
        m_motorLeft = new SparkMax(ElevatorConstants.MOTOR_LEFT_CANID, ElevatorConstants.MOTOR_LEFT_TYPE);
        m_motorRight = new SparkMax(ElevatorConstants.MOTOR_RIGHT_CANID, ElevatorConstants.MOTOR_RIGHT_TYPE);

        // Configure left motor
        m_configLeft = new SparkMaxConfig();
        m_configLeft
                .inverted(ElevatorConstants.MOTOR_LEFT_IS_INVERTED)
                .smartCurrentLimit(ElevatorConstants.MOTOR_SMART_CURRENT_LIMIT);

        // Set encoder conversion factors
        m_configLeft.encoder
                .positionConversionFactor(ElevatorConstants.MOTOR_REVOLUTION_PER_INCH)
                .velocityConversionFactor(ElevatorConstants.MOTOR_REVOLUTION_PER_INCH);

        // Configure closed-loop control settings
        m_configLeft.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(ElevatorConstants.SLOT_ZERO_P, ClosedLoopSlot.kSlot0)
                .i(ElevatorConstants.SLOT_ZERO_I, ClosedLoopSlot.kSlot0)
                .d(ElevatorConstants.SLOT_ZERO_D, ClosedLoopSlot.kSlot0)
                .p(ElevatorConstants.SLOT_ONE_P, ClosedLoopSlot.kSlot1)
                .i(ElevatorConstants.SLOT_ONE_I, ClosedLoopSlot.kSlot1)
                .d(ElevatorConstants.SLOT_ONE_D, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1).maxMotion
                .maxAcceleration(ElevatorConstants.SLOT_ZERO_MAX_ACCELERATION, ClosedLoopSlot.kSlot0)
                .maxVelocity(ElevatorConstants.SLOT_ZERO_MAX_VELOCITY, ClosedLoopSlot.kSlot0)
                .maxAcceleration(ElevatorConstants.SLOT_ONE_MAX_ACCELERATION, ClosedLoopSlot.kSlot1)
                .maxVelocity(ElevatorConstants.SLOT_ONE_MAX_VELOCITY, ClosedLoopSlot.kSlot1);

        // Apply left motor configuration
        m_motorLeft.configure(m_configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure right motor to follow the left motor
        m_configRight = new SparkMaxConfig();
        m_configRight.follow(m_motorLeft, ElevatorConstants.MOTOR_RIGHT_IS_INVERTED);
        m_motorRight.configure(m_configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize encoders and closed-loop controllers
        m_encoderLeft = m_motorLeft.getEncoder();
        m_encoderRight = m_motorRight.getEncoder();
        m_controllerLeft = m_motorLeft.getClosedLoopController();
        m_controllerRight = m_motorRight.getClosedLoopController();

        // Set initial elevator height from encoder
        m_elevatorHeight = m_encoderLeft.getPosition();

        this.setDefaultCommand(new RunCommand(() -> this.setWantedState(m_currentElevatorState), this));
    }

    // Override the periodic method to update elevator logic
    @Override
    public void periodic() {
        // Update elevator height from encoder
        m_elevatorHeight = m_encoderLeft.getPosition();

        // Send telemetry data to SmartDashboard
        this.sendSmartDashboardValues();
    }

    // Method to send elevator state and encoder values to SmartDashboard
    private void sendSmartDashboardValues() {
        SmartDashboard.putString("Elevator State", m_currentElevatorState.getName());
        SmartDashboard.putString("Elevator Motor State", m_currentMotorState.getName());
        SmartDashboard.putNumber("Elevator Desired Height", m_currentElevatorState.getHeight());
        SmartDashboard.putNumber("Elevator Encoder Reading", m_elevatorHeight);
        SmartDashboard.putNumber("Elevator Motor Left Amps", m_motorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Motor Right Amps", m_motorRight.getOutputCurrent());
    }

    // Method to update the desired elevator state
    public void setWantedState(ElevatorState wantedState) {
        m_currentElevatorState = wantedState;

        if (DriverStation.isEnabled() && !(m_currentElevatorState == ElevatorState.NO_POWER
                || m_currentElevatorState == ElevatorState.DISABLED)) {
            if (ElevatorConstants.TEST_ANTI_MOTOR_CRASH_IS_ENABLED) {
                this.determineMotorState();
            }
            m_controllerLeft.setReference(
                    m_currentElevatorState.getHeight(), ControlType.kMAXMotionPositionControl,
                    m_currentMotorState.getClosedLoopSlot(),
                    m_currentMotorState.getFF(),
                    m_currentMotorState.getArbFFUnits());
        } else {
            m_motorLeft.stopMotor(); // Stop motor if disabled
        }
    }

    // Method to Determine Motor State for Anti Crash System
    private void determineMotorState() {
        // Check if the elevator is at CORAL_LEVEL_FOUR or CORAL_INTAKE state
        if ((m_currentElevatorState == ElevatorState.CORAL_LEVEL_FOUR
                || m_currentElevatorState == ElevatorState.CORAL_INTAKE)
                // Check if elevator height falls within the anti-crash safety threshold
                && (m_elevatorHeight >= ElevatorState.CORAL_LEVEL_FOUR.getHeight()
                        * ElevatorConstants.ANTI_CRASH_PERCENT_FROM_TOP
                        || m_elevatorHeight <= ElevatorState.CORAL_INTAKE.getHeight()
                                * ElevatorConstants.ANTI_CRASH_PERCENT_ABOVE_BOT)) {
            // Set motor state to ANTI_CRASH to prevent potential collisions
            m_currentMotorState = MotorState.ANTI_CRASH;
        } else {
            // Otherwise, set motor state to NORMAL
            m_currentMotorState = MotorState.NORMAL;
        }

    }

}

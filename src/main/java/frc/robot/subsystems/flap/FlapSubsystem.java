package frc.robot.subsystems.flap;

// Import necessary libraries and dependencies
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.FlapConstants;

// Define the FlapSubsystem class which extends SubsystemBase
public class FlapSubsystem extends SubsystemBase {

    // Declare motor, configuration, encoder, and controller variables
    private final SparkMax m_motor;
    private final SparkMaxConfig m_config;
    private final AbsoluteEncoder m_encoder;
    private final SparkClosedLoopController m_controller;
    private double m_flapAngle = -291.00; // Default angle for debugging

    // Enum to represent different flap states
    public enum FlapState {
        UP(FlapConstants.ANGLE_UP, "UP"),
        DOWN(FlapConstants.ANGLE_DOWN, "DOWN"),
        DISABLED(FlapConstants.ANGLE_DISBALED, "DISABLED");

        private final double angle;
        private final String name;

        // Constructor for FlapState enum
        FlapState(double angle, String name) {
            this.angle = angle;
            this.name = name;
        }

        // Getter method for angle
        public double getAngle() {
            return angle;
        }

        // Getter method for name
        public String getName() {
            return name;
        }
    }

    // Declare and initialize current flap state
    public FlapState m_currentFlapState = FlapState.DISABLED;

    // Constructor for FlapSubsystem
    public FlapSubsystem() {
        // Initialize motor with CAN ID and motor type from constants
        m_motor = new SparkMax(FlapConstants.MOTOR_CANID, FlapConstants.MOTOR_TYPE);

        // Configure motor settings
        m_config = new SparkMaxConfig();
        m_config
                .inverted(FlapConstants.MOTOR_IS_INVERTED) // Set motor inversion
                .smartCurrentLimit(FlapConstants.MOTOR_SMART_CURRENT_LIMIT); // Set current limit

        // Configure encoder conversion factors
        m_config.encoder
                .positionConversionFactor(FlapConstants.MOTOR_REVOLUTION_PER_INCH)
                .velocityConversionFactor(FlapConstants.MOTOR_REVOLUTION_PER_INCH);

        // Configure closed-loop control settings
        m_config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(FlapConstants.SLOT_ZERO_P, ClosedLoopSlot.kSlot0)
                .i(FlapConstants.SLOT_ZERO_I, ClosedLoopSlot.kSlot0)
                .d(FlapConstants.SLOT_ZERO_D, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0).maxMotion
                .maxAcceleration(FlapConstants.SLOT_ZERO_MAX_ACCELERATION, ClosedLoopSlot.kSlot0)
                .maxVelocity(FlapConstants.SLOT_ZERO_MAX_VELOCITY, ClosedLoopSlot.kSlot0);

        // Apply the motor configuration with safe reset and persistence options
        m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize encoder and closed-loop controller
        m_encoder = m_motor.getAbsoluteEncoder();
        m_controller = m_motor.getClosedLoopController();

        // Get initial flap angle from encoder
        m_flapAngle = m_encoder.getPosition();
    }

    // Override the periodic method in the SubsystemBase to update motor logic
    @Override
    public void periodic() {
        // Update flap angle from encoder
        m_flapAngle = m_encoder.getPosition();

        // Control logic: If robot is enabled and flap state is not disabled, set
        // reference position
        if (DriverStation.isEnabled() && !((m_currentFlapState == FlapState.DISABLED))) {
            m_controller.setReference(m_currentFlapState.getAngle(), ControlType.kMAXMotionPositionControl,
                    ClosedLoopSlot.kSlot0,
                    FlapConstants.SLOT_ZERO_FF,
                    FlapConstants.SLOT_ZERO_FF_UNITS);
        } else {
            // Stop motor if robot is disabled or flap state is disabled
            m_motor.stopMotor();
        }

        // Send data to SmartDashboard for monitoring
        this.sendSmartDashboardValues();
    }

    // Method to send flap state and encoder values to SmartDashboard
    private void sendSmartDashboardValues() {
        SmartDashboard.putString("Flap State", m_currentFlapState.getName());
        SmartDashboard.putNumber("Angle Encoder Reading", m_flapAngle);
    }

    // Method to update the desired flap state
    public void setWantedState(FlapState wantedState) {
        m_currentFlapState = wantedState;
    }
}

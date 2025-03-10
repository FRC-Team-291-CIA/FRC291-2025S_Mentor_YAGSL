package frc.robot.subsystems.coral;

// Import necessary libraries and dependencies
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;

// Define the CoralSubsystem class, extending SubsystemBase
public class CoralSubsystem extends SubsystemBase {

    // Declare motor controllers, configurations, and intake sensor
    private final SparkMax m_motorLeft, m_motorRight;
    private final SparkMaxConfig m_configLeft, m_configRight;
    private final DigitalInput m_intakeSensor;

    // Store intake sensor value
    public boolean m_intakeSensorValue = false;

    // Constructor for CoralSubsystem
    public CoralSubsystem() {
        // Initialize motors for left and right coral system
        m_motorLeft = new SparkMax(CoralConstants.MOTOR_LEFT_CANID, CoralConstants.MOTOR_LEFT_TYPE);
        m_motorRight = new SparkMax(CoralConstants.MOTOR_RIGHT_CANID, CoralConstants.MOTOR_RIGHT_TYPE);

        // Configure left motor
        m_configLeft = new SparkMaxConfig();
        m_configLeft.inverted(CoralConstants.MOTOR_LEFT_IS_INVERTED);

        // Apply configuration to left motor
        m_motorLeft.configure(m_configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure right motor to follow the left motor
        m_configRight = new SparkMaxConfig();
        m_configRight.follow(m_motorLeft, ElevatorConstants.MOTOR_RIGHT_IS_INVERTED);

        // Apply configuration to right motor
        m_motorRight.configure(m_configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize intake sensor
        m_intakeSensor = new DigitalInput(CoralConstants.INTAKE_SENSOR_DIOPORT);
    }

    // Override periodic method to update intake sensor value
    @Override
    public void periodic() {
        // Update intake sensor value
        m_intakeSensorValue = this.getIntakeSensorValue();

        // Send updated values to SmartDashboard
        this.sendSmartDashboardValues();
    }

    // Method to send sensor data to SmartDashboard
    private void sendSmartDashboardValues() {
        SmartDashboard.putBoolean("Intake Sensor Value", m_intakeSensorValue);
    }

    // Method to retrieve intake sensor value
    public boolean getIntakeSensorValue() {
        // Return the sensor reading, inverting if necessary
        if (CoralConstants.INTAKE_SENSOR_IS_INVERTED) {
            return !m_intakeSensor.get();
        } else {
            return m_intakeSensor.get();
        }
    }

    // Method to provide a BooleanSupplier for the intake sensor
    public BooleanSupplier getIntakeSensorSupplier() {
        return this::getIntakeSensorValue;
    }

    // Method to set the motor speed
    public void setSpeed(double newSpeed) {
        m_motorLeft.set(newSpeed);
    }

    // Command to move coral forward at a slow speed
    public Command MANUAL_FORWARD_SLOW() {
        return new RunCommand(() -> m_motorLeft.set(CoralConstants.SPEED_MANUAL_FORWARD_SLOW), this);
    }

    // Command to move coral forward at a fast speed
    public Command MANUAL_FORWARD_FAST() {
        return new RunCommand(() -> m_motorLeft.set(CoralConstants.SPEED_MANUAL_FORWARD_FAST), this);
    }

    // Command to move coral in reverse at a slow speed
    public Command MANUAL_REVERSE_SLOW() {
        return new RunCommand(() -> m_motorLeft.set(CoralConstants.SPEED_MANUAL_REVERSE_SLOW), this);
    }

    // Command to move coral in reverse at a fast speed
    public Command MANUAL_REVERSE_FAST() {
        return new RunCommand(() -> m_motorLeft.set(CoralConstants.SPEED_MANUAL_REVERSE_FAST), this);
    }
}

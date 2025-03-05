package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {

    public final SparkMax m_motorLeft, m_motorRight;
    public final SparkMaxConfig m_configLeft, m_configRight;

    public final DigitalInput m_intakeSensor;

    public enum CoralState {
        MANUAL_FORWARD_SLOW,
        MANUAL_FORWARD_FAST,
        MANUAL_REVERSE_SLOW,
        MANUAL_REVERSE_FAST,
        AUTOMATIC_INTAKE,
        AUTOMATIC_SCORE;
    }

    public CoralSubsystem() {
        m_motorLeft = new SparkMax(CoralConstants.MOTOR_LEFT_CANID, CoralConstants.MOTOR_LEFT_TYPE);
        m_motorRight = new SparkMax(CoralConstants.MOTOR_RIGHT_CANID, CoralConstants.MOTOR_RIGHT_TYPE);

        m_configLeft = new SparkMaxConfig();
        m_configLeft
                .inverted(CoralConstants.MOTOR_LEFT_IS_INVERTED);

        m_configRight = new SparkMaxConfig();
        m_configRight
                .inverted(CoralConstants.MOTOR_RIGHT_IS_INVERTED);

        m_intakeSensor = new DigitalInput(CoralConstants.INTAKE_SENSOR_DIOPORT);
    }

    @Override
    public void periodic() {

    }

    public BooleanSupplier getIntakeSensorSupplier() {
        return this::getIntakeSensorValue;
    }

    public boolean getIntakeSensorValue() {
        if (CoralConstants.INTAKE_SENSOR_IS_INVERTED) {
            return !m_intakeSensor.get();
        } else {
            return m_intakeSensor.get();
        }
    }
}

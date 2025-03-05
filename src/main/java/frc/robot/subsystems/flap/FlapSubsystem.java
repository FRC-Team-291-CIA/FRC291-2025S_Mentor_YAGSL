package frc.robot.subsystems.flap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.Constants.FlapConstants;

public class FlapSubsystem extends SubsystemBase {

    private final SparkMax m_motor;
    private final SparkMaxConfig m_config;
    private final AbsoluteEncoder m_encoder;
    private final SparkClosedLoopController m_controller;

    public enum FlapState {
        UP,
        DOWN,
        DISABLED;
    }

    public FlapSubsystem() {
        m_motor = new SparkMax(FlapConstants.MOTOR_CANID, FlapConstants.MOTOR_TYPE);

        m_config = new SparkMaxConfig();
        m_config
                .inverted(FlapConstants.MOTOR_IS_INVERTED);

        m_encoder = m_motor.getAbsoluteEncoder();

        m_controller = m_motor.getClosedLoopController();
    }

    @Override
    public void periodic() {

    }

}

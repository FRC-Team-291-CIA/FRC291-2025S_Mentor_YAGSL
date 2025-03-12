package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax m_motor;
    private final SparkMaxConfig m_config;

    public ClimberSubsystem() {
        m_motor = new SparkMax(ClimberConstants.MOTOR_CANID, ClimberConstants.MOTOR_TYPE);

        m_config = new SparkMaxConfig();

        m_config.inverted(ClimberConstants.MOTOR_IS_INVERTED);

        m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}

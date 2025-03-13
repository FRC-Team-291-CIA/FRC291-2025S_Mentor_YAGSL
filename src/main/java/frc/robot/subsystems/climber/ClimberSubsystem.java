package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax m_motor;
    private final SparkMaxConfig m_config;

    public ClimberSubsystem() {
        m_motor = new SparkMax(ClimberConstants.MOTOR_CANID, ClimberConstants.MOTOR_TYPE);

        m_config = new SparkMaxConfig();

        m_config
                .inverted(ClimberConstants.MOTOR_IS_INVERTED)
                .smartCurrentLimit(ClimberConstants.MOTOR_SMART_CURRENT_LIMIT)
                .idleMode(ClimberConstants.MOTOR_MODE);

        m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.setDefaultCommand(run(() -> m_motor.stopMotor()).withName("DEFAULT: STOPPED"));
    }

    @Override
    public void periodic() {
        // Send updated values to SmartDashboard
        this.sendSmartDashboardValues();
    }

    // Method to send sensor data to SmartDashboard
    private void sendSmartDashboardValues() {
        SmartDashboard.putNumber("Climber Motor Current", m_motor.getOutputCurrent());
    }

    public void in() {
        this.m_motor.set(ClimberConstants.SPEED_IN);
    }

    public void out() {
        this.m_motor.set(ClimberConstants.SPEED_OUT);
    }

}

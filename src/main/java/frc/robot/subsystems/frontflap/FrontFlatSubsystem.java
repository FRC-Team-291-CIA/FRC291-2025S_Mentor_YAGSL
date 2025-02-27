package frc.robot.subsystems.frontflap;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FrontFlapConstants;

public class FrontFlatSubsystem extends SubsystemBase {

    private SparkMax motor;

    private SparkMaxConfig motorConfig;

    public FrontFlatSubsystem() {
        motor = new SparkMax(FrontFlapConstants.motorCANID, FrontFlapConstants.motorType);

        motorConfig = new SparkMaxConfig();

        motorConfig.inverted(FrontFlapConstants.motorInvert);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.setDefaultCommand(this.cStop().withName("DEFAULT: STOP"));
    }

    public void vStop() {
        motor.stopMotor();
    }

    public Command cStop() {
        return run(
                () -> {
                    this.vStop();
                })
                .withName("STOP");
    }

    public void vUp() {
        motor.set(FrontFlapConstants.speed);
    }

    public Command cIn() {
        return run(
                () -> {
                    this.vUp();
                })
                .withName("UP");
    }

    public void vDown() {
        motor.set(FrontFlapConstants.speed * -1);
    }

    public Command cOut() {
        return run(
                () -> {
                    this.vDown();
                })
                .withName("DOWN");
    }

}

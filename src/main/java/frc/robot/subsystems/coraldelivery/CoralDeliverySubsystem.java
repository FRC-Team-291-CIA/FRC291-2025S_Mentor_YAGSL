package frc.robot.subsystems.coraldelivery;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CoralConstants;

public class CoralDeliverySubsystem extends SubsystemBase {

    private SparkMax motorLeft, motorRight;

    private SparkMaxConfig motorLeftConfig, motorRightConfig;

    private DigitalInput breakBeam;

    public CoralDeliverySubsystem() {
        motorLeft = new SparkMax(CoralConstants.motorLeftCANID, CoralConstants.motorLeftType);
        motorRight = new SparkMax(CoralConstants.motorRightCANID, CoralConstants.motorRightType);

        motorLeftConfig = new SparkMaxConfig();
        motorRightConfig = new SparkMaxConfig();

        motorLeftConfig.inverted(CoralConstants.motorLeftInvert);
        motorRightConfig.inverted(CoralConstants.motorRightInvert);

        motorLeft.configure(motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorRight.configure(motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.setDefaultCommand(this.cStop().withName("DEFAULT: STOP"));
    }

    public void vStop() {
        motorLeft.stopMotor();
        motorRight.stopMotor();
    }

    public Command cStop() {
        return run(
                () -> {
                    this.vStop();
                })
                .withName("STOP");
    }

    public void vIn() {
        motorLeft.set(CoralConstants.inSpeed);
        motorRight.set(CoralConstants.inSpeed);
    }

    public Command cIn() {
        return run(
                () -> {
                    this.vIn();
                })
                .withName("IN");
    }

    public void vOut() {
        motorLeft.set(CoralConstants.outSpeed);
        motorRight.set(CoralConstants.outSpeed);
    }

    public Command cOut() {
        return run(
                () -> {
                    this.vOut();
                })
                .withName("OUT");
    }

}

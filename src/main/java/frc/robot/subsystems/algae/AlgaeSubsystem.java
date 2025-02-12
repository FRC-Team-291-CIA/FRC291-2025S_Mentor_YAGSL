package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {

    private SparkMax motorLeft, motorRight;

    private SparkMaxConfig motorLeftConfig, motorRightConfig;

    public AlgaeSubsystem() {
        motorLeft = new SparkMax(AlgaeConstants.motorLeftCANID, AlgaeConstants.motorLeftType);
        motorRight = new SparkMax(AlgaeConstants.motorRightCANID, AlgaeConstants.motorRightType);
    
        motorLeftConfig = new SparkMaxConfig();
        motorRightConfig = new SparkMaxConfig();

        motorLeftConfig.inverted(AlgaeConstants.motorLeftInvert);
        motorRightConfig.inverted(AlgaeConstants.motorRightInvert);

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
        motorLeft.set(AlgaeConstants.inSpeed);
        motorRight.set(AlgaeConstants.inSpeed);
    }

    public Command cIn() {
        return run(
                () -> {
                    this.vIn();
                })
                .withName("IN");
    }

    public void vOut() {
        motorLeft.set(AlgaeConstants.outSpeed);
        motorRight.set(AlgaeConstants.outSpeed);
    }

    public Command cOut() {
        return run(
                () -> {
                    this.vOut();
                })
                .withName("OUT");
    }

}

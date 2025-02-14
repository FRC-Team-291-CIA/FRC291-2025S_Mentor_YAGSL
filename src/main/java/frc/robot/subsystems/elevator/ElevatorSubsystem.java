package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax motorLeft, motorRight;
    private SparkMaxConfig motorLeftConfig, motorRightConfig;
    private SparkClosedLoopController closedLoopControllerLeft;
    private RelativeEncoder sparkEncoderLeft;

    public ElevatorSubsystem() {
        motorLeft = new SparkMax(ElevatorConstants.motorLeftCANID, ElevatorConstants.motorLeftType);
        motorRight = new SparkMax(ElevatorConstants.motorRightCANID, ElevatorConstants.motorRightType);

        motorLeftConfig = new SparkMaxConfig();

        motorLeftConfig.inverted(ElevatorConstants.motorLeftInvert);

        motorLeftConfig.encoder
                .positionConversionFactor(0.9);

        motorLeftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.4)
                .i(0.0)
                .d(0.0)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.1, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        motorLeft.configure(motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        closedLoopControllerLeft = motorLeft.getClosedLoopController();

        sparkEncoderLeft = motorLeft.getEncoder();

        motorRightConfig = new SparkMaxConfig();

        motorRightConfig.follow(motorLeft, ElevatorConstants.motorRightInvert);

        motorRight.configure(motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.setDefaultCommand(cStop().withName("DEFAULT: STOP"));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Left Encoder", sparkEncoderLeft.getPosition());
        SmartDashboard.putNumber("Elevator Left Motor Current", motorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Right Motor Current", motorRight.getOutputCurrent());
    }

    public void vStop() {
        motorLeft.stopMotor();
    }

    public Command cStop() {
        return run(
                () -> {
                    motorLeft.stopMotor();
                })
                .withName("STOP");
    }

    public void vGoTo(double targetHeight) {
        if (sparkEncoderLeft.getPosition() > targetHeight) {
            closedLoopControllerLeft.setReference(targetHeight, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else {
            closedLoopControllerLeft.setReference(targetHeight, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        }

    }

    public Command cGoTo(double targetHeight) {
        return run(
                () -> {
                    this.vGoTo(targetHeight);
                })
                .withName("GO TO");
    }

}

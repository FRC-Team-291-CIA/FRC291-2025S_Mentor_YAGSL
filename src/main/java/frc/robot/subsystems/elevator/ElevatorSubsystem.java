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

import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax motorLeft, motorRight;
    private final SparkMaxConfig motorLeftConfig, motorRightConfig;
    private final SparkClosedLoopController closedLoopControllerLeft;
    private final RelativeEncoder sparkEncoderLeft;

    private double leftEncoderReading;

    public enum ElevatorState {
        CORAL_LEVEL_FOUR(ElevatorConstants.HEIGHT_CORAL_LEVEL_FOUR, "LEVEL 4"),
        CORAL_LEVEL_THREE(
                ElevatorConstants.HEIGHT_CORAL_LEVEL_THREE, "LEVEL 3"),
        CORAL_LEVEL_TWO(
                ElevatorConstants.HEIGHT_CORAL_LEVEL_TWO, "LEVEL 2"),
        CORAL_LEVEL_ONE(
                ElevatorConstants.HEIGHT_CORAL_LEVEL_ONE, "LEVEL 1"),
        CORAL_INTAKE(
                ElevatorConstants.HEIGHT_CORAL_INTAKE, "INTAKE"),
        STOWED(ElevatorConstants.HEIGHT_STOWED, "STOWED"),
        PARK(ElevatorConstants.HEIGHT_PARK, "PARK"),
        NO_POWER(ElevatorConstants.HEIGHT_NO_POWER, "NO POWER");

        private final double height;
        private final String name;

        ElevatorState(double height, String name) {
            this.height = height;
            this.name = name;
        }

        public double getHeight() {
            return height;
        }

        public String getName() {
            return name;
        }
    }

    public ElevatorState currentState;

    private enum MotorState {
        MOVING_UP(ClosedLoopSlot.kSlot0, "MOVING UP"),
        HOLDING(ClosedLoopSlot.kSlot1, "HOLDING"),
        MOVING_DOWN(ClosedLoopSlot.kSlot0, "MOVING DOWN");

        private final ClosedLoopSlot closedLoopSlot;
        private final String name;

        MotorState(ClosedLoopSlot closedLoopSlot, String name) {
            this.closedLoopSlot = closedLoopSlot;
            this.name = name;
        }

        public ClosedLoopSlot getClosedLoopSlot() {
            return closedLoopSlot;
        }

        public String getName() {
            return name;
        }
    }

    public MotorState motorState;

    public ElevatorSubsystem() {
        motorLeft = new SparkMax(ElevatorConstants.motorLeftCANID, ElevatorConstants.motorLeftType);
        motorRight = new SparkMax(ElevatorConstants.motorRightCANID, ElevatorConstants.motorRightType);

        motorLeftConfig = new SparkMaxConfig();

        motorLeftConfig.inverted(ElevatorConstants.motorLeftInvert);

        double motorRevolutionsPerInch = (4335.00 / 4698.00);

        motorLeftConfig.encoder
                .positionConversionFactor(motorRevolutionsPerInch)
                .velocityConversionFactor(motorRevolutionsPerInch);

        motorLeftConfig.smartCurrentLimit(40);

        motorLeftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(ElevatorConstants.P, ClosedLoopSlot.kSlot0)
                .i(ElevatorConstants.I, ClosedLoopSlot.kSlot0)
                .d(ElevatorConstants.D, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        motorLeftConfig.closedLoop.maxMotion
                .maxAcceleration(960, ClosedLoopSlot.kSlot0)
                .maxVelocity(2880, ClosedLoopSlot.kSlot0);

        motorLeft.configure(motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        closedLoopControllerLeft = motorLeft.getClosedLoopController();

        sparkEncoderLeft = motorLeft.getEncoder();

        leftEncoderReading = sparkEncoderLeft.getPosition();

        motorRightConfig = new SparkMaxConfig();

        motorRightConfig.smartCurrentLimit(40);

        motorRightConfig.follow(motorLeft, ElevatorConstants.motorRightInvert);

        motorRight.configure(motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.currentState = ElevatorState.NO_POWER;

        this.motorState = MotorState.MOVING_UP;

        this.setDefaultCommand(Commands.run(() -> this.setWantedState(currentState), this));
    }

    @Override
    public void periodic() {
        this.leftEncoderReading = sparkEncoderLeft.getPosition();

        SmartDashboard.putNumber("Elevator Left Encoder Reading", this.leftEncoderReading);
        SmartDashboard.putNumber("Elevator Wanted Height", currentState.getHeight());

        SmartDashboard.putString("Elevator Current State", currentState.getName());
        SmartDashboard.putString("Elevator Motor State", motorState.getName());

        SmartDashboard.putNumber("Elevator Left Motor Current", motorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Right Motor Current", motorRight.getOutputCurrent());
    }

    public void setWantedState(ElevatorState wantedState) {
        if (wantedState != currentState) {
            this.motorState = updateMotorState(wantedState);
            this.currentState = wantedState;
        }
        if (currentState != ElevatorState.NO_POWER) {
            closedLoopControllerLeft.setReference(currentState.getHeight(), ControlType.kMAXMotionPositionControl,
                    this.motorState.getClosedLoopSlot());
        } else {
            motorLeft.stopMotor();
        }
    }

    private MotorState updateMotorState(ElevatorState wantedState) {
        if (this.leftEncoderReading > wantedState.getHeight()) {
            return MotorState.MOVING_UP;
        } else {
            return MotorState.MOVING_DOWN;
        }
    }

}

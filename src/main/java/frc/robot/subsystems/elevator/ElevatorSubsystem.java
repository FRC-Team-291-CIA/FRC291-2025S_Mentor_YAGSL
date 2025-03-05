package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    public final SparkMax m_motorLeft, m_motorRight;
    public final SparkMaxConfig m_configLeft, m_configRight;
    public final RelativeEncoder m_encoderLeft, m_encoderRight;
    public final SparkClosedLoopController m_controllerLeft, m_controllerRight;

    private double m_elevatorHeight = -291.00;

    public enum ElevatorState {
        CORAL_LEVEL_FOUR(ElevatorConstants.HEIGHT_CORAL_LEVEL_FOUR, "CORAL LEVEL 4"),
        CORAL_LEVEL_THREE(
                ElevatorConstants.HEIGHT_CORAL_LEVEL_THREE, "CORAL LEVEL 3"),
        CORAL_LEVEL_TWO(
                ElevatorConstants.HEIGHT_CORAL_LEVEL_TWO, "CORAL LEVEL 2"),
        CORAL_LEVEL_ONE(
                ElevatorConstants.HEIGHT_CORAL_LEVEL_ONE, "CORAL LEVEL 1"),
        CORAL_INTAKE(
                ElevatorConstants.HEIGHT_CORAL_INTAKE, "CORAL INTAKE"),
        PARK(ElevatorConstants.HEIGHT_PARK, "PARK"),
        NO_POWER(ElevatorConstants.HEIGHT_KILL_POWER, "KILL POWER"),
        DISABLED(ElevatorConstants.HEIGHT_DISABLED, "DISABLED");

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

    private enum MotorState {
        NORMAL(ClosedLoopSlot.kSlot0, ElevatorConstants.SLOT_ONE_FF_UNITS, ElevatorConstants.SLOT_ONE_FF, "NORMAL"),
        ANTI_CRASH(ClosedLoopSlot.kSlot1, ElevatorConstants.SLOT_ONE_FF_UNITS, ElevatorConstants.SLOT_ONE_FF,
                "ANTI CRASH");

        private final ClosedLoopSlot closedLoopSlot;
        private final ArbFFUnits arbFFUnits;
        private final double ff;
        private final String name;

        MotorState(ClosedLoopSlot closedLoopSlot, ArbFFUnits arbFFUnits, double ff, String name) {
            this.closedLoopSlot = closedLoopSlot;
            this.arbFFUnits = arbFFUnits;
            this.ff = ff;
            this.name = name;
        }

        public ClosedLoopSlot getClosedLoopSlot() {
            return closedLoopSlot;
        }

        public ArbFFUnits getArbFFUnits() {
            return arbFFUnits;
        }

        public double getFF() {
            return ff;
        }

        public String getName() {
            return name;
        }
    }

    public ElevatorState currentElevatorState = ElevatorState.DISABLED;
    private MotorState currentMotorState = MotorState.NORMAL;

    public ElevatorSubsystem() {
        m_motorLeft = new SparkMax(ElevatorConstants.MOTOR_LEFT_CANID, ElevatorConstants.MOTOR_LEFT_TYPE);
        m_motorRight = new SparkMax(ElevatorConstants.MOTOR_RIGHT_CANID, ElevatorConstants.MOTOR_RIGHT_TYPE);

        m_configLeft = new SparkMaxConfig();
        m_configLeft
                .inverted(ElevatorConstants.MOTOR_LEFT_IS_INVERTED)
                .smartCurrentLimit(ElevatorConstants.MOTOR_SMART_CURRENT_LIMIT);

        m_configLeft.encoder
                .positionConversionFactor(ElevatorConstants.MOTOR_REVOLUTION_PER_INCH)
                .velocityConversionFactor(ElevatorConstants.MOTOR_REVOLUTION_PER_INCH);

        m_configLeft.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(ElevatorConstants.SLOT_ZERO_P, ClosedLoopSlot.kSlot0)
                .i(ElevatorConstants.SLOT_ZERO_I, ClosedLoopSlot.kSlot0)
                .d(ElevatorConstants.SLOT_ZERO_D, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
                .p(ElevatorConstants.SLOT_ONE_P, ClosedLoopSlot.kSlot1)
                .i(ElevatorConstants.SLOT_ONE_I, ClosedLoopSlot.kSlot1)
                .d(ElevatorConstants.SLOT_ONE_D, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1).maxMotion
                .maxAcceleration(ElevatorConstants.SLOT_ZERO_MAX_ACCELERATION, ClosedLoopSlot.kSlot0)
                .maxVelocity(ElevatorConstants.SLOT_ZERO_MAX_VELOCITY, ClosedLoopSlot.kSlot0)
                .maxAcceleration(ElevatorConstants.SLOT_ONE_MAX_ACCELERATION, ClosedLoopSlot.kSlot1)
                .maxVelocity(ElevatorConstants.SLOT_ONE_MAX_VELOCITY, ClosedLoopSlot.kSlot1);

        m_configRight = new SparkMaxConfig();
        m_configRight
                .inverted(ElevatorConstants.MOTOR_RIGHT_IS_INVERTED);

        m_encoderLeft = m_motorLeft.getEncoder();
        m_encoderRight = m_motorRight.getEncoder();

        m_controllerLeft = m_motorLeft.getClosedLoopController();
        m_controllerRight = m_motorRight.getClosedLoopController();
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled() && !(currentElevatorState == ElevatorState.NO_POWER)) {
            this.determineMotorState();
            m_controllerLeft.setReference(currentElevatorState.getHeight(), ControlType.kMAXMotionPositionControl,
                    currentMotorState.getClosedLoopSlot(), currentMotorState.getFF(),
                    currentMotorState.getArbFFUnits());
        } else {
            m_motorLeft.stopMotor();
        }
    }

    public void setWantedState(ElevatorState wantedState) {
        currentElevatorState = wantedState;
    }

    public void determineMotorState() {
        if ((currentElevatorState == ElevatorState.CORAL_LEVEL_FOUR
                || currentElevatorState == ElevatorState.CORAL_INTAKE)
                && (m_elevatorHeight >= ElevatorState.CORAL_LEVEL_FOUR.getHeight() * 0.9
                        || m_elevatorHeight <= ElevatorState.CORAL_INTAKE.getHeight() * 1.1)) {
            currentMotorState = MotorState.ANTI_CRASH;
        } else {
            currentMotorState = MotorState.NORMAL;
        }
    }

}

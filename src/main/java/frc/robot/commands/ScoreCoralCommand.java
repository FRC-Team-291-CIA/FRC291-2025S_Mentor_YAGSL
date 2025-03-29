package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;

import frc.robot.Constants.CIAAutoConstants;

/**
 * Command for controlling the intake of a "coral" object in the robot.
 * The command follows a sequence:
 * 1. Run intake at an initial speed until the sensor detects the coral.
 * 2. Adjust speed once the coral is detected.
 * 3. Stop the intake once the coral has passed through.
 */
public class ScoreCoralCommand extends Command {

    private final CoralSubsystem m_coralSubsystem; // Reference to the coral intake subsystem
    private final ElevatorSubsystem m_elevatorSubsystem;
    private ElevatorState m_elevatorLevel; // Desired elevator level for the coral intake
    private boolean m_commandDone; // Flag to End
    private Timer m_timer;

    private enum STAGE {
        STAGE_ONE,
        STAGE_TWO;
    }

    private STAGE m_currentStage;

    /**
     * Constructor for the intake command.
     * 
     * @param coralSubsystem The subsystem responsible for handling the coral
     *                       intake.
     */
    public ScoreCoralCommand(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
            ElevatorState elevatorLevel) {
        this.m_coralSubsystem = coralSubsystem;
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_elevatorLevel = elevatorLevel;

        this.m_timer = new Timer();

        // Declare subsystem dependencies to prevent conflicts with other commands.
        this.addRequirements(coralSubsystem, m_elevatorSubsystem);
    }

    /**
     * Initializes the command when first scheduled.
     * Resets the tracking flags to ensure a fresh start.
     */
    @Override
    public void initialize() {
        m_currentStage = STAGE.STAGE_ONE;
        m_commandDone = false;

        m_timer.reset();
        m_timer.start();

        System.out.println("SCORE CORAL COMMAND: " + m_elevatorLevel.getName());
        System.out.println(m_currentStage.toString());
    }

    /**
     * This method runs repeatedly while the command is active.
     * It controls the speed of the intake mechanism based on sensor feedback.
     */
    @Override
    public void execute() {
        switch (m_currentStage) {
            case STAGE_ONE:
                if (m_timer.get() > 1) {
                    m_currentStage = STAGE.STAGE_TWO;
                    System.out.println(m_currentStage.toString());
                    m_timer.reset();
                    m_timer.start();
                } else {
                    m_elevatorSubsystem.setWantedState(m_elevatorLevel);
                }
                break;
            case STAGE_TWO:
                if (m_timer.get() > 1) {
                    m_commandDone = true;
                } else {
                    m_elevatorSubsystem.setWantedState(m_elevatorLevel);
                    m_coralSubsystem.setSpeed(0.5);
                }
                break;
        }
    }

    /**
     * Runs when the command is interrupted or finishes normally.
     * Ensures the intake is stopped.
     * 
     * @param interrupted True if another command interrupted this one.
     */
    @Override
    public void end(boolean interrupted) {
        System.out.println("SCORE CORAL COMMAND: COMMAND COMPLETE");
        m_coralSubsystem.setSpeed(0.00); // Stop the intake motor
        m_elevatorSubsystem.setWantedState(ElevatorState.CORAL_INTAKE);
    }

    /**
     * Determines when the command should finish.
     * 
     * @return True when the coral has been fully processed.
     */
    @Override
    public boolean isFinished() {
        return m_commandDone;
    }
}

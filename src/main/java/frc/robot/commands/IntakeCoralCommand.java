package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.coral.CoralSubsystem;

import frc.robot.Constants.CIAAutoConstants;

/**
 * Command for controlling the intake of a "coral" object in the robot.
 * The command follows a sequence:
 * 1. Run intake at an initial speed until the sensor detects the coral.
 * 2. Adjust speed once the coral is detected.
 * 3. Stop the intake once the coral has passed through.
 */
public class IntakeCoralCommand extends Command {

    private final CoralSubsystem m_coralSubsystem; // Reference to the coral intake subsystem
    private boolean m_commandDone; // Flag to End
    private Timer m_timer;

    private enum STAGE {
        STAGE_ONE("STAGE ONE"),
        STAGE_TWO("STAGE TWO"),
        STAGE_THREE("STAGE THREE");

        private final String name;

        // Constructor for STAGE enum
        STAGE(String name) {
            this.name = name;
        }

        // Getter method for name
        public String getName() {
            return name;
        }
    }

    private STAGE m_currentStage;

    /**
     * Constructor for the intake command.
     * 
     * @param coralSubsystem The subsystem responsible for handling the coral
     *                       intake.
     */
    public IntakeCoralCommand(CoralSubsystem coralSubsystem) {
        this.m_coralSubsystem = coralSubsystem;

        this.m_timer = new Timer();

        // Declare subsystem dependencies to prevent conflicts with other commands.
        this.addRequirements(coralSubsystem);
    }

    /**
     * Initializes the command when first scheduled.
     * Resets the tracking flags to ensure a fresh start.
     */
    @Override
    public void initialize() {
        m_currentStage = STAGE.STAGE_ONE;
        m_commandDone = false;
        System.out.println("INTAKE CORAL COMMAND");
        System.out.println(m_currentStage.getName());
    }

    /**
     * This method runs repeatedly while the command is active.
     * It controls the speed of the intake mechanism based on sensor feedback.
     */
    @Override
    public void execute() {
        switch (m_currentStage) {
            case STAGE_ONE:
                if (m_coralSubsystem.m_intakeSensorValue) {
                    m_currentStage = STAGE.STAGE_TWO;
                    System.out.println(m_currentStage.getName());
                } else {
                    m_coralSubsystem.setSpeed(CIAAutoConstants.AUTO_SPEED_CORAL_BEFORE_ENTER);
                }
                break;
            case STAGE_TWO:
                if (!m_coralSubsystem.m_intakeSensorValue) {
                    m_currentStage = STAGE.STAGE_THREE;
                    m_timer.reset();
                    m_timer.start();
                    System.out.println("STAGE THREE");
                } else {
                    m_coralSubsystem.setSpeed(CIAAutoConstants.AUTO_SPEED_CORAL_AFTER_ENTER);
                }
                break;
            case STAGE_THREE:
                if (m_coralSubsystem.m_intakeSensorValue) {
                    m_commandDone = true;
                    System.out.println("DONE");
                } else {
                    m_coralSubsystem.setSpeed(-CIAAutoConstants.AUTO_SPEED_CORAL_AFTER_ENTER);
                }

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
        m_coralSubsystem.setSpeed(0.00); // Stop the intake motor
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

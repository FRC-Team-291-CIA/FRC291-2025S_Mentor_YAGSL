package frc.robot.commands;

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
    private boolean hasCoralEntered, hasCoralExited, commandDone; // Flags to track the coral's position and command
                                                                  // completion

    /**
     * Constructor for the intake command.
     * 
     * @param coralSubsystem The subsystem responsible for handling the coral
     *                       intake.
     */
    public IntakeCoralCommand(CoralSubsystem coralSubsystem) {
        this.m_coralSubsystem = coralSubsystem;

        // Declare subsystem dependencies to prevent conflicts with other commands.
        this.addRequirements(coralSubsystem);
    }

    /**
     * Initializes the command when first scheduled.
     * Resets the tracking flags to ensure a fresh start.
     */
    @Override
    public void initialize() {
        hasCoralEntered = false;
        hasCoralExited = false;
        commandDone = false;
    }

    /**
     * This method runs repeatedly while the command is active.
     * It controls the speed of the intake mechanism based on sensor feedback.
     */
    @Override
    public void execute() {
        if (!hasCoralEntered) {
            // Start running the intake at an initial speed
            m_coralSubsystem.setSpeed(CIAAutoConstants.SPEED_AUTO_CORAL_BEFORE_ENTER);

            // Check if the coral has entered (detected by sensor)
            hasCoralEntered = m_coralSubsystem.m_intakeSensorValue;
        } else if (hasCoralEntered && !hasCoralExited) {
            // Adjust speed after the coral has entered
            m_coralSubsystem.setSpeed(CIAAutoConstants.SPEED_AUTO_CORAL_AFTER_ENTER);

            // Check if the coral has exited (sensor no longer detects it)
            hasCoralExited = !m_coralSubsystem.m_intakeSensorValue;
        } else if (hasCoralEntered && hasCoralExited) {
            // Stop the intake once the coral has fully passed through
            m_coralSubsystem.setSpeed(0.00);
            commandDone = true; // Mark the command as completed
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
        return commandDone;
    }
}

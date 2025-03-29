package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.flap.FlapSubsystem;
import frc.robot.subsystems.flap.FlapSubsystem.FlapState;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.CIAAutoConstants;

/**
 * Command for controlling the intake of a "coral" object in the robot.
 * The command follows a sequence:
 * 1. Run intake at an initial speed until the sensor detects the coral.
 * 2. Adjust speed once the coral is detected.
 * 3. Stop the intake once the coral has passed through.
 */
public class AlgaeFlapCommand extends Command {

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final FlapSubsystem m_flapSubsystem;
    private final SwerveSubsystem m_drivebase;;
    private ElevatorState m_elevatorLevel; // Desired elevator level for the coral intake
    private boolean m_commandDone; // Flag to End
    private Timer m_timer;

    private enum STAGE {
        STAGE_ONE,
        STAGE_TWO,
        STAGE_THREE,
        STAGE_FOUR;
    }

    private STAGE m_currentStage;

    /**
     * Constructor for the intake command.
     * 
     * @param coralSubsystem The subsystem responsible for handling the coral
     *                       intake.
     */
    public AlgaeFlapCommand(ElevatorSubsystem elevatorSubsystem, FlapSubsystem flapSubsystem, SwerveSubsystem drivebase,
            ElevatorState elevatorLevel) {
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_elevatorLevel = elevatorLevel;

        this.m_flapSubsystem = flapSubsystem;
        this.m_drivebase = drivebase;

        this.m_timer = new Timer();

        // Declare subsystem dependencies to prevent conflicts with other commands.
        this.addRequirements(m_elevatorSubsystem, m_flapSubsystem, m_drivebase);
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
                    m_flapSubsystem.setWantedState(FlapState.UP);
                }
                break;
            case STAGE_TWO:
                if (m_timer.get() > 1) {
                    m_currentStage = STAGE.STAGE_THREE;
                    System.out.println(m_currentStage.toString());
                    m_timer.reset();
                    m_timer.start();
                } else {
                    m_elevatorSubsystem.setWantedState(m_elevatorLevel);
                    m_flapSubsystem.setWantedState(FlapState.LEVEL);
                }
                break;
            case STAGE_THREE:
                if (m_timer.get() > 0.5) {
                    m_currentStage = STAGE.STAGE_FOUR;
                    System.out.println(m_currentStage.toString());
                    m_timer.reset();
                    m_timer.start();
                } else {
                    m_elevatorSubsystem.setWantedState(m_elevatorLevel);
                    m_flapSubsystem.setWantedState(FlapState.UP);
                }
                break;
            case STAGE_FOUR:
                if (m_timer.get() > 0.5) {
                    m_commandDone = true; // Mark the command as done
                } else {
                    m_elevatorSubsystem.setWantedState(m_elevatorLevel);
                    m_flapSubsystem.setWantedState(FlapState.UP);
                    m_drivebase.drive(new ChassisSpeeds(-5, 0, 0)); // Reverse the robot to clear the coral
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

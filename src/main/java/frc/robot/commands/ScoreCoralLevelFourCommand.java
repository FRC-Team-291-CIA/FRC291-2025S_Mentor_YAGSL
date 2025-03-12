package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;

import frc.robot.Constants.CIAAutoConstants;

public class ScoreCoralLevelFourCommand extends Command {

    private final CoralSubsystem m_coralSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private boolean commandDone;

    public ScoreCoralLevelFourCommand(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.m_coralSubsystem = coralSubsystem;
        this.m_elevatorSubsystem = elevatorSubsystem;

        this.addRequirements(m_coralSubsystem, m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
        commandDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_coralSubsystem.setSpeed(0.00);
        m_elevatorSubsystem.setWantedState(ElevatorState.CORAL_INTAKE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return commandDone;
    }
}

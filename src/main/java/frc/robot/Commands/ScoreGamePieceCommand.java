package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ScoringLocation;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;


//should account for piece (cone--y dir offset for placement, cube), scoring location (bottom, mid, high)
public class ScoreGamePieceCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;

    public ScoreGamePieceCommand(ElevatorSubsystem elevator, IntakeSubsystem intake) {
        this.elevator = elevator;
        this.intake = intake;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }


    @Override
    public boolean isFinished() {
       return true;
    }


    @Override
    public void end(boolean isFinished) {
    }
}

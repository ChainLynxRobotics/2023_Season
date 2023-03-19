package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

//arm up, extends elevator to desired setpoint and releases gamepiece, on end calls toStartConfigCommand
public class ScoreGamePieceCommand extends CommandBase {

    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private ArmSubsystem arm;
    private int setpointLocation;

    public ScoreGamePieceCommand(ElevatorSubsystem elevator, IntakeSubsystem intake, ArmSubsystem arm, int setpointLocation) {
        this.elevator = elevator;
        this.intake = intake;
        this.arm = arm;
        this.setpointLocation = setpointLocation;

        addRequirements(elevator, intake, arm);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new InstantCommand(arm::expand)
            .andThen(new ElevatorCommand(elevator, intake, setpointLocation)
            .andThen(new ReleaseCommand(intake, 0.8))));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean isFinished) {
        CommandScheduler.getInstance().schedule(new ToStartConfigCommand(arm, elevator, intake));
    }
}

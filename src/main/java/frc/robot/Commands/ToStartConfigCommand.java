package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class ToStartConfigCommand extends CommandBase {

    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;

    //arm down, turn on intake
    public ToStartConfigCommand(ArmSubsystem arm, ElevatorSubsystem elevator, IntakeSubsystem intake) {
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;

        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new InstantCommand(arm::retract));
        CommandScheduler.getInstance().schedule(new ElevatorCommand(elevator, intake, 0));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ToStartConfigCommand extends CommandBase {

    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;

    //arm down, turn on intake
    public ToStartConfigCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
        this.arm = arm;
        this.elevator = elevator;

        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}
}

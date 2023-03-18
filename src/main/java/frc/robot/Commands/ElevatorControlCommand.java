package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorControlCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private Joystick stick;

    public ElevatorControlCommand(Joystick stick, ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.stick = stick;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.simpleMovement(stick.getRawAxis(1));
    } 
}

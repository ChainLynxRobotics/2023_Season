package frc.robot.Commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private double setpoint;

    public ElevatorCommand(ElevatorSubsystem elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
    }

    
    @Override
    public void end(boolean interrupted) {
    }
}

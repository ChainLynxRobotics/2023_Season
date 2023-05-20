package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class AutoElevatorCommand extends CommandBase {

    private ElevatorSubsystem elevator;
    private double setpoint;
    private double startTime;

    public AutoElevatorCommand(ElevatorSubsystem elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }
  
    @Override
    public void execute() {
        elevator.setElevatorSetpoint(setpoint);

        if (setpoint-elevator.getDrivingEncoder().getPosition() > ElevatorConstants.MAX_TRAVEL_LIMIT) {
            elevator.setElevatorSetpoint(setpoint-ElevatorConstants.ELEVATOR_RAMP_DIST);

            if (System.currentTimeMillis()-startTime > 300) {
                elevator.setElevatorSetpoint(setpoint);
            }
        } else if (setpoint-elevator.getDrivingEncoder().getPosition() < -ElevatorConstants.MAX_TRAVEL_LIMIT) {
            elevator.setElevatorSetpoint(setpoint+ElevatorConstants.ELEVATOR_RAMP_DIST);

            if (System.currentTimeMillis()-startTime > 300) {
                elevator.setElevatorSetpoint(setpoint);
            }
        } else {
            elevator.setElevatorSetpoint(setpoint);
        }
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(setpoint - elevator.getDrivingEncoder().getPosition()) < 0.05) {
            return true;
        }
        return false;
    }
}

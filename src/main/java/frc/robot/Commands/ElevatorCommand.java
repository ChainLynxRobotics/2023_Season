package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {

    private ElevatorSubsystem elevator;
    private double setpoint;
    private double startTime;

    public ElevatorCommand(ElevatorSubsystem elevator, double setpoint) {
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
        elevator.moveElevator(setpoint);

        if (setpoint-elevator.getDrivingEncoder().getPosition() > ElevatorConstants.MAX_TRAVEL_LIMIT) {
            elevator.moveElevator(setpoint-ElevatorConstants.ELEVATOR_RAMP_DIST);

            if (System.currentTimeMillis()-startTime > 300) {
                elevator.moveElevator(setpoint);
            }
        } else if (setpoint-elevator.getDrivingEncoder().getPosition() < -ElevatorConstants.MAX_TRAVEL_LIMIT) {
            elevator.moveElevator(setpoint+ElevatorConstants.ELEVATOR_RAMP_DIST);

            if (System.currentTimeMillis()-startTime > 300) {
                elevator.moveElevator(setpoint);
            }
        } else {
            elevator.moveElevator(setpoint);
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

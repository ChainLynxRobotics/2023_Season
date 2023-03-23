package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class AutoElevatorCommand extends CommandBase {

    private ElevatorSubsystem elevator;
    private double setpoint;


    public AutoElevatorCommand(ElevatorSubsystem elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.moveElevator(setpoint);

        System.out.printf("setting setpoint via command, setpoint at: %f \n", setpoint);
        SmartDashboard.putNumber("Elevator Setpoint (rotations)", setpoint);

        if (setpoint-elevator.getDrivingEncoder().getPosition() > 8) {
            elevator.moveElevator(setpoint-ElevatorConstants.ELEVATOR_RAMP_DIST);
            double startTime = System.currentTimeMillis();

            if (System.currentTimeMillis()-startTime > 100) {
                elevator.moveElevator(setpoint);
            }
        } else if (setpoint-elevator.getDrivingEncoder().getPosition() < -8) {
            elevator.moveElevator(setpoint+ElevatorConstants.ELEVATOR_RAMP_DIST);
            double startTime2 = System.currentTimeMillis();

            if (System.currentTimeMillis()-startTime2 > 300) {
                elevator.moveElevator(setpoint+5);
            }
        } else {
            elevator.moveElevator(setpoint);
        }
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(setpoint - elevator.getDrivingEncoder().getPosition()) < 1) {
            System.out.println("finished!");
            return true;
        }
        return false;
    }
}

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Bindings;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeGamePiece;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class ElevatorCommand extends CommandBase {

    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private double setpoint;
    private int setpointLocation;

    public ElevatorCommand(ElevatorSubsystem elevator, IntakeSubsystem intake, int setpointLocation) {
        this.elevator = elevator;
        this.intake = intake;
        this.setpointLocation = setpointLocation;
        this.setpoint = 0;

        addRequirements(elevator, intake);
    }


    @Override
    public void execute() {
        if (setpointLocation == 15) {
            setpoint = 7.5;
        } else if (setpointLocation == 13) {
            setpoint = 3;
        } else if (setpointLocation == 12) {
            setpoint = 7.5;
        } else if (setpointLocation == 11) {
            if (intake.getState() == IntakeGamePiece.CONE) {
                setpoint = 15.5;
            } else {
                setpoint = 13.5;
            }
        } else if (setpointLocation == 16) {
            setpoint = 14;
        } else if (setpointLocation == 14) {
            setpoint = 0;
        }

        System.out.printf("setting setpoint via command, setpoint at: %f \n", setpoint);
        SmartDashboard.putNumber("Elevator Setpoint (rotations)", setpoint);

        if (Math.abs(setpoint-elevator.getDrivingEncoder().getPosition()) > 8) {
            elevator.moveElevator(setpoint-ElevatorConstants.ELEVATOR_RAMP_DIST);
            double startTime = System.currentTimeMillis();

            if (System.currentTimeMillis()-startTime > 100) {
                elevator.moveElevator(setpoint);
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

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
        if (setpointLocation == Bindings.groundPickUp) {
            setpoint = ElevatorConstants.groundPickupCubeHybrid;
        } else if (setpointLocation == Bindings.lowScoreElevatorSetpoint) {
            setpoint = ElevatorConstants.coneDrivingWithLift;
        } else if (setpointLocation == Bindings.midScoreElevatorSetpoint) {
            setpoint = ElevatorConstants.midElevatorGamepiece;
        } else if (setpointLocation == Bindings.highScoreElevatorSetpoint) {
            if (intake.getState() == IntakeGamePiece.CONE) {
                setpoint = ElevatorConstants.highElevatorConeSetpoint;
            } else {
                setpoint = ElevatorConstants.highElevatorCubeSetpoint;
            }
        } else if (setpointLocation == Bindings.doubleSubstationSetpoint) {
            setpoint = ElevatorConstants.doubleSubstationSetpoint;
        } else if (setpointLocation == Bindings.fullRetraction) {
            setpoint = ElevatorConstants.fullRetractionSetpoint;
        }

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

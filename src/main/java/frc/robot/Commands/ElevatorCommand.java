package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Bindings;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class ElevatorCommand extends CommandBase {

    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private double setpoint;
    private int setpointLocation;

    double startTime;

    public ElevatorCommand(ElevatorSubsystem elevator, IntakeSubsystem intake, int setpointLocation) {
        this.elevator = elevator;
        this.intake = intake;
        this.setpointLocation = setpointLocation;
        this.setpoint = 0;

        addRequirements(elevator, intake);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();

    }

    //TODO: fix elevator setpoint logic (not broken, but directly pass in setpoint it's too verbose)
    @Override
    public void execute() {
        if (setpointLocation == Bindings.groundPickUp) {
            setpoint = ElevatorConstants.groundPickupCubeHybrid;
        } else if (setpointLocation == Bindings.lowScoreElevatorSetpoint) {
            setpoint = ElevatorConstants.coneDrivingWithLift;
        } else if (setpointLocation == Bindings.midScoreElevatorSetpoint) {
            setpoint = ElevatorConstants.midElevatorGamepiece;
        } else if (setpointLocation == Bindings.highScoreElevatorSetpoint) {
            if (intake.getState() == GamePiece.CONE) {
                setpoint = ElevatorConstants.highElevatorConeSetpoint;
            } else {
                setpoint = ElevatorConstants.highElevatorCubeSetpoint;
            }
        } else if (setpointLocation == Bindings.doubleSubstationSetpoint) {
            setpoint = ElevatorConstants.doubleSubstationSetpoint;
        } else if (setpointLocation == Bindings.fullRetraction) {
            setpoint = ElevatorConstants.fullRetractionSetpoint;
        }

        //to avoid elevator slamming and excessive acceleration, ramp according to "triangular" velocity motion profile
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

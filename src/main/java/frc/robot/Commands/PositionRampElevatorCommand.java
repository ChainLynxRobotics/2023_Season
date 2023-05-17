package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.MotionProfiles.PositionRamping;
import frc.robot.Constants.Bindings;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class PositionRampElevatorCommand extends CommandBase {

    private ElevatorSubsystem elevator;
    private double setpoint;
    private PositionRamping positionRamp;

    double startTime;

    public PositionRampElevatorCommand(ElevatorSubsystem elevator, IntakeSubsystem intake, int setpointLocation, double timeRampingMillis) {
        this.elevator = elevator;
        this.setpoint = 0;

        addRequirements(elevator, intake);

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

        positionRamp = new PositionRamping(setpoint, timeRampingMillis, elevator.getDrivingEncoder().getPosition());
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();

    }

    //TODO: fix elevator setpoint logic (not broken, but directly pass in setpoint it's too verbose)
    @Override
    public void execute() {
        
        elevator.setElevatorSetpoint(positionRamp.evaluate(System.currentTimeMillis() - startTime));
        
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(setpoint - elevator.getDrivingEncoder().getPosition()) < 0.05) {
            return true;
        }
        return false;
    }
}

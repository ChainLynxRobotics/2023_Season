package frc.robot.Commands;

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
        setpoint = 0;

        addRequirements(elevator, intake);
    }

    @Override
    public void initialize() {
       switch(setpointLocation) {
        case Bindings.groundPickUp:
            setpoint = ElevatorConstants.groundPickupCubeHybrid;
        case Bindings.lowScoreElevatorSetpoint:
            setpoint = ElevatorConstants.coneDrivingWithLift;
        case Bindings.midScoreElevatorSetpoint:
            setpoint = ElevatorConstants.midElevatorGamepiece;
        case Bindings.highScoreElevatorSetpoint:
            if (intake.getState() == IntakeGamePiece.CONE) {
                setpoint = ElevatorConstants.highElevatorConeSetpoint;
            } else {
                setpoint = ElevatorConstants.highElevatorCubeSetpoint;
            }
        case Bindings.doubleSubstationSetpoint:
                setpoint = ElevatorConstants.doubleSubstationSetpoint;
        default:
            setpoint = 0;
       }
    }

    @Override
    public void execute() {
        elevator.moveElevator(setpoint);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(setpoint - elevator.getDrivingEncoder().getPosition()) < 0.1) {
            return true;
        }
        return false;
    }
}

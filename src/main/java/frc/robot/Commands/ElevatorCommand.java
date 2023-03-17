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
        if (setpointLocation == Bindings.fullRetractionSetpoint) {
            setpoint = ElevatorConstants.fullRetractionSetpoint;
        } else if (setpointLocation == Bindings.lowElevatorSetpoint) {
            setpoint = ElevatorConstants.lowElevatorSetpoint;
        } else if (setpoint == Bindings.doubleSubstationSetpoint) {
            setpoint = ElevatorConstants.doubleSubstationSetpoint;
        }

        if (intake.getState() == IntakeGamePiece.CUBE) {
            if (setpoint == Bindings.midElevatorSetpoint) {
                setpoint = ElevatorConstants.midElevatorCubeSetpoint;
            } else if (setpoint == Bindings.highElevatorSetpoint) {
                setpoint = ElevatorConstants.highElevatorCubeSetpoint;
            }
        } else if (intake.getState() == IntakeGamePiece.CONE) {
            if (setpoint == Bindings.midElevatorSetpoint) {
                setpoint = ElevatorConstants.midElevatorConeSetpoint;
            } else if (setpoint == Bindings.highElevatorSetpoint) {
                setpoint = ElevatorConstants.highElevatorConeSetpoint;
            }
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

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class ElevatorCommand extends CommandBase {

    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private double setpoint;
    private double startTime;

    public ElevatorCommand(ElevatorSubsystem elevator, IntakeSubsystem intake, double setpoint) {
        this.elevator = elevator;
        this.intake = intake;
        this.setpoint = setpoint;

        addRequirements(elevator, intake);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();

        /*check if wrong setpoint and adjust; in RobotContainer the state is only checked on initialization
        one button is bound to 2 setpoints depending on intake state*/
        if (intake.getState() == GamePiece.CONE && setpoint == ElevatorConstants.highElevatorCubeSetpoint) {
            setpoint = ElevatorConstants.highElevatorConeSetpoint;
        } else if (intake.getState() == GamePiece.CUBE && setpoint == ElevatorConstants.highElevatorConeSetpoint) {
            setpoint = ElevatorConstants.highElevatorCubeSetpoint;
        }
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

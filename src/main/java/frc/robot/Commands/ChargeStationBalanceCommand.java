package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.utils.ChargeStationStates;
import frc.utils.ChargeStationStates.States;

/*
 * moves forward at constant initial speed until pitch changes
 * if pitch > 0, move forward with + scale factor proportional to error
 * if < 0, move forward wtih - scale factor proportional to error
 */
public class ChargeStationBalanceCommand extends CommandBase {

    private DriveSubsystem drive;
    private ElevatorSubsystem elevator;

    private ChargeStationStates balancer = new ChargeStationStates();

    private Timer timer;

    private static final double ELEVATOR_SETPOINT = 0;
    private boolean pitchChanged = false;

    public ChargeStationBalanceCommand(DriveSubsystem drive, ElevatorSubsystem elevator) {
        this.drive = drive;
        this.elevator = elevator;
        timer = new Timer();

        addRequirements(drive, elevator);

        SmartDashboard.putNumber("ChargeStationBalance/CurrentAngle", 0);
        SmartDashboard.putNumber("ChargeStationBalance/AngularError", 0);
    }

    @Override
    public void initialize() {
        System.out.println("charge station init");
        timer.reset();
        timer.start();

        pitchChanged = false;

        elevator.moveElevator(ELEVATOR_SETPOINT);
    }

    @Override
    public void execute() {
        //get pitch and error
        double pitch = drive.getPitch();
        double speed = balancer.autoBalanceRoutine();

        SmartDashboard.putNumber("ChargeStationBalance/CurrentAngle", pitch);

       
        if (pitch > AutoConstants.MIN_PITCH_CHANGE) {
            pitchChanged = true;
        }

        if (pitchChanged) {
            drive.mainDrive(-1*speed, 0, 0);
        } else {
            drive.mainDrive(-0.6,0,0);
        }

        if (Math.abs(pitch) > 6 && balancer.getState() != States.ON_CHARGE_STATION) {
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(1)) {
            System.out.println("command terminated");
            timer.stop();
            return true;
        }
        return false;
    }
}

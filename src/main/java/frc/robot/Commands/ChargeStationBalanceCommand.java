package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;

/*
 * moves forward at constant initial speed until pitch changes
 * if pitch > 0, move forward with + scale factor proportional to error
 * if < 0, move forward wtih - scale factor proportional to error
 */
public class ChargeStationBalanceCommand extends CommandBase {

    private DriveSubsystem drive;
    private ElevatorSubsystem elevator;

    private Timer timer;

    private static final double ERROR = 2;
    private static final double TIME_ENGAGED = 1;
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
        timer.reset();
        timer.start();

        elevator.moveElevator(ELEVATOR_SETPOINT);
    }

    @Override
    public void execute() {
        //get pitch and error
        double pitch = drive.getPitch();
        double error = ERROR - pitch;

        SmartDashboard.putNumber("ChargeStationBalance/CurrentAngle", pitch);
        SmartDashboard.putNumber("ChargeStationBalance/AngularEffort", error);

        //drive and put data on smart dashboard
        if (pitch > 2) {
            pitchChanged = true;
            drive.mainDrive(0.1 * pitch, 0, 0);
        } else if (pitch < -2) {
            drive.mainDrive(-0.1 * pitch, 0, 0);
        }

        //reset timer if robot is not balanced
        if (error > ERROR || pitchChanged == false) {
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return (timer.get() >= TIME_ENGAGED) ? true : false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}

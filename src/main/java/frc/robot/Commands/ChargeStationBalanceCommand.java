package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.utils.ChargeStationStates;

/*
 * moves forward at constant initial speed until pitch changes
 * if pitch > 0, move forward with + scale factor proportional to error
 * if < 0, move forward wtih - scale factor proportional to error
 */
public class ChargeStationBalanceCommand extends CommandBase {

    private DriveSubsystem drive;
    private ElevatorSubsystem elevator;
    private Joystick stick;

    private ChargeStationStates balancer = new ChargeStationStates();

    private Timer timer;

    private static final double ELEVATOR_SETPOINT = 0;
    private boolean pitchChanged = false;

    public ChargeStationBalanceCommand(DriveSubsystem drive, ElevatorSubsystem elevator, Joystick stick) {
        this.drive = drive;
        this.elevator = elevator;
        this.stick = stick;
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
        System.out.println("pitch at " + pitch);

       
        if (pitch > 5) {
            pitchChanged = true;
        }

        if (pitchChanged) {
            drive.mainDrive(-1.2*speed, 0, 0);
        } else {
            drive.mainDrive(-0.5,0,0);
        }

        if (Math.abs(pitch) > 1) {
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        if (stick.getRawButton(9) || timer.hasElapsed(1)) {
            return true;
        }
        return false;
    }
}

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.utils.SwerveUtils;

//gets gyro heading and rotates chassis until 180 degree offset from initial heading is reached
public class Rotate180Command extends CommandBase {
    private DriveSubsystem drivetrain;
    private double initGyroHeading;
    private double curGyroHeading;

    public Rotate180Command(DriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }  

    @Override
    public void initialize() {
        initGyroHeading = drivetrain.getHeading();
        curGyroHeading = drivetrain.getHeading();
    }

    @Override
    public void execute() {
        drivetrain.mainDrive(0, 0, 0.1*SwerveUtils.AngleDifference(curGyroHeading, (initGyroHeading+180)%360));
        curGyroHeading = drivetrain.getHeading();
    }
    
    @Override
    public boolean isFinished() {
        if (Math.abs(initGyroHeading+180-curGyroHeading) < 2) {
            return true;
        }
        return false;
    }
}

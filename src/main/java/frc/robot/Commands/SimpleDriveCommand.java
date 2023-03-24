package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class SimpleDriveCommand extends CommandBase {

    private DriveSubsystem drive;
    private XboxController controller;
    private double multiplier = 1;

    public SimpleDriveCommand(DriveSubsystem drive, XboxController controller) {
        this.drive = drive;
        this.controller = controller;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        boolean slowMode = false;
        if (controller.getLeftTriggerAxis() != 0) {
            multiplier = 2;
            slowMode = false;
        } else if (controller.getRightTriggerAxis() != 0) {
            multiplier = 0.5;
            slowMode = true;
        }

        SmartDashboard.putBoolean("slow mode", slowMode);


        drive.drive(
            MathUtil.applyDeadband(
                -multiplier * controller.getLeftY(),
                DriveConstants.DRIVE_DEADBAND
            ),
            MathUtil.applyDeadband(
                -multiplier * controller.getLeftX(),
                DriveConstants.DRIVE_DEADBAND
            ),
            MathUtil.applyDeadband(
                -multiplier * controller.getRightX(),
                DriveConstants.DRIVE_DEADBAND
            ),
            MathUtil.applyDeadband(
                -multiplier * controller.getRightY(),
                DriveConstants.DRIVE_DEADBAND
            ),
            controller.getRightBumper(),
            controller.getAButton()
        );
    }
}

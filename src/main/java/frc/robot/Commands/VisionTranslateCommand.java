package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import org.photonvision.PhotonUtils;

public class VisionTranslateCommand extends CommandBase {

    //private VisionSubsystem vision;
    private VisionSubsystem vision;
    private DriveSubsystem drive;
    private XboxController controller;
    private double kp = 0.2;
    private double ki = 10;
    private double kd = 0.05;

    private PIDController forwardController;

    public VisionTranslateCommand(
        VisionSubsystem vision,
        DriveSubsystem drive,
        XboxController controller
    ) {
        this.vision = vision;
        this.drive = drive;

        this.controller = controller;
        SmartDashboard.putNumber("kP", kp);
        SmartDashboard.putNumber("kI", ki);
        SmartDashboard.putNumber("kD", kd);
        forwardController = new PIDController(kp, ki, kd);

        addRequirements(vision, drive);

        forwardController.setIntegratorRange(0, 2);
    }

    @Override
    public void execute() {
        double forwardSpeed = -controller.getLeftY();

        if (vision.getHasTarget()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.CAMERA_HEIGHT,
                Constants.TARGET_HEIGHT,
                Constants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(vision.getBestTarget().getPitch())
            );

            forwardSpeed =
                forwardController.calculate(range, Constants.GOAL_RANGE);
        } else {
            forwardSpeed = 0;
        }

        drive.mainDrive(
            -MathUtil.applyDeadband(forwardSpeed, 0.06),
            MathUtil.applyDeadband(controller.getLeftX(), 0.06),
            MathUtil.applyDeadband(controller.getRightX(), 0.06)
        );
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(forwardController.getPositionError()) < 1) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, 0, false, false);
    }
}

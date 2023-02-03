package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.SimpleDriveCommand;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.VisionSubsystem;

public class RobotContainer {
    private Joystick joystick = new Joystick(Constants.JOYSTICK_PORT1);
    private Joystick armJoystick = new Joystick(Constants.JOYSTICK_PORT2);

    private final Drivetrain drivetrain;
    private final ArmSubsystem arm;
    private final VisionSubsystem vision;

    private final PowerDistribution powerDistribution = new PowerDistribution();
    private final AutoUtils autoUtils = new AutoUtils();
    
    public RobotContainer() {
        powerDistribution.clearStickyFaults();

        drivetrain = new Drivetrain();
        arm = new ArmSubsystem();
        vision = new VisionSubsystem();

        startDefaultCommands();

    }

    public void startDefaultCommands() {
        drivetrain.setDefaultCommand(new SimpleDriveCommand(drivetrain, 
        () -> -joystick.getRawAxis(1),
        () -> -joystick.getRawAxis(0)));

        arm.setDefaultCommand(new RunCommand(() -> arm.moveArm(() -> -armJoystick.getRawAxis(0)), arm));
    }


    public Drivetrain getRobotDrive() {
        return drivetrain;
    }

    public AutoUtils getAutoPath() {
        return autoUtils;
    }

    public Command getAutonomousCommand() {
        return new RunCommand(() -> drivetrain.testDrive(0.1, 0.1), drivetrain).withTimeout(5);
    }
}

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants.ShooterZone;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooter;

    private boolean goalAtTop;
    private ShooterZone zone;
    private double angleSetpoint;

    private double startTime;
    
    public ShooterCommand(boolean goalAtTop, ShooterZone zone) {
        this.goalAtTop = goalAtTop;
        this.zone = zone;
        this.angleSetpoint = shooter.characterizeZone(zone)[2];
        startTime = System.currentTimeMillis();

        shooter = new ShooterSubsystem(goalAtTop);

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (goalAtTop) {
            shooter.setAngle(zone, 0);
        }
    }

    @Override
    public void execute() {
        if (!goalAtTop) {
            shooter.setAngle(zone, angleSetpoint);
        }
        if (shooter.isAtSetpoint(angleSetpoint, shooter.ticksToAngle(shooter.getHoodMotor().getEncoder().getPosition()))) {
            shooter.setRPM(zone);
        }
    }

    @Override
    public boolean isFinished() {
        double curTime = System.currentTimeMillis();
        if (shooter.isAtSetpoint(shooter.getLaunchVelocity().get(zone), shooter.toMetersPerSec(shooter.getHoodMotor().getEncoder().getVelocity())) ||
            (curTime-startTime) > 10000) {
                return true;
        }
        return false;
    }
}

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants.ShooterZone;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooter;

    private boolean goalAtTop;
    private ShooterZone zone;
    private boolean shortRange;
    private double angleSetpoint;

    private double startTime;
    
    public ShooterCommand(boolean goalAtTop, ShooterZone zone, boolean shortRange, double angleSetpoint) {
        this.goalAtTop = goalAtTop;
        this.zone = zone;
        this.shortRange = shortRange;
        this.angleSetpoint = angleSetpoint;
        startTime = System.currentTimeMillis();

        shooter = new ShooterSubsystem(goalAtTop);

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (goalAtTop) {
            shooter.setAngle(shortRange, 0);
        }
    }

    @Override
    public void execute() {
        if (!goalAtTop) {
            shooter.setAngle(shortRange, angleSetpoint);
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

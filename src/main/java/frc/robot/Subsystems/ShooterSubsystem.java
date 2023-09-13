package frc.robot.Subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterZone;


public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax flywheelMotor;
    private SparkMaxPIDController pidController;

    private Optional<DoubleSolenoid> binaryHoodController;
    private Optional<CANSparkMax> hoodController;

    private boolean goalAtTop;
    

    private Map<ShooterZone, Double> shooterEffort = new HashMap<>();

    public ShooterSubsystem(boolean goalAtTop) {
        flywheelMotor = new CANSparkMax(ShooterConstants.SHOOTER_FLYWHEEL_PORT, MotorType.kBrushless);
        pidController = flywheelMotor.getPIDController();
        pidController.setP(ShooterConstants.kP);
        pidController.setI(ShooterConstants.kI);
        pidController.setD(ShooterConstants.kD);
        pidController.setFF(ShooterConstants.kFF);

        this.goalAtTop = goalAtTop;

        if (goalAtTop) {
            binaryHoodController = Optional.empty();
            hoodController = Optional.of(new CANSparkMax(ShooterConstants.SHOOTER_HOOD_PORT, MotorType.kBrushless));
        } else {
            binaryHoodController = Optional.of(new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_forward1, Constants.SOLENOID_reverse1));
            hoodController = Optional.empty();
        }
        
    }

    public void setRPM(ShooterZone zone) {
        double initVel = shooterEffort.get(zone);
        updateRPM(initVel);
    }

 
    public void updateRPM(double setpointVel) {
        pidController.setReference(setpointVel, ControlType.kVelocity);
    }

    public void setAngle(ShooterZone zone, double setpoint) {
        if (goalAtTop) {
            updateAngle(setpoint);
        } else {
            if (zone == ShooterZone.ZONE3) {
                binaryHoodController.get().set(Value.kReverse);
            } else {
                binaryHoodController.get().set(Value.kForward);
            }
        }
    }

    //angular pid
    public void updateAngle(double setpoint) {
        double error = setpoint - ticksToAngle(hoodController.get().getEncoder().getPosition());
        double effort = ShooterConstants.kP*error;
        hoodController.get().set(effort);
    }

    public boolean isAtSetpoint(double setpoint, double current) {
        if (Math.abs(setpoint - current) < ShooterConstants.SETPOINT_ERROR) {
            return true;
        }
        return false;
    }

    //convert encoder rotations to angle
    public double ticksToAngle(double ticks) {
        return ticks*ShooterConstants.TICKS_PER_ROTATION/360;
    }

    public double toMetersPerSec(double rpm) {
        return rpm*2*Math.PI*ShooterConstants.FLYWHEEL_RADIUS;
    }


    @Override
    public void periodic() {
        //log values to smart dashboard here
    }


    public void addShooterZone(ShooterZone zone) {
        double[] zoneVals = characterizeZone(zone);
        double horDist = zoneVals[0];
        double vertDist = zoneVals[1];
        double angle = goalAtTop ? zoneVals[2] : Math.atan(vertDist/horDist);

        double initVel;
        if (!goalAtTop) {
            initVel = Math.sqrt(2)*Math.sqrt(-9.81*horDist/(Math.sin(angle)*Math.cos(angle)) + 9.81*vertDist/Math.pow(Math.sin(angle), 2) - 
            9.81*Math.sqrt(3*horDist*horDist*Math.pow(Math.sin(angle),2) - 2*horDist*vertDist*Math.sin(angle)*Math.cos(angle) + 
            vertDist*vertDist*Math.pow(Math.cos(angle),2)/(Math.pow(Math.sin(angle),2)*Math.cos(angle))))/2;

            shooterEffort.put(ShooterZone.ZONE1, velocityToRPM(initVel)); //placeholder zone
        } else {
            initVel = Math.sqrt(2*9.81*vertDist)/Math.sin(angle);
        }
        
    }

    public double velocityToRPM(double velocity) {
        return velocity*(2*Math.PI/60)*ShooterConstants.FLYWHEEL_RADIUS;
    }

    //replace placeholder values
    public double[] characterizeZone(ShooterZone zone) {
        switch(zone) {
            case ZONE1:
                if (goalAtTop) {
                    return new double[]{10,5,Math.atan(5/10)};
                }
                return new double[]{10,5,45};
            case ZONE2:
                if (goalAtTop) {
                    return new double[]{15,5,Math.atan(5/15)};
                }
                return new double[]{15,5,45};
            case ZONE3:
                if (goalAtTop) {
                    return new double[]{10,5,Math.atan(1)};
                }
                return new double[]{5,5,60};
            default:
                return new double[]{0,0,0};
        }
    }

    public CANSparkMax getHoodMotor() {
        return hoodController.get();
    }

    public CANSparkMax getFlywheelMotor() {
        return flywheelMotor;
    }

    public  Map<ShooterZone, Double> getLaunchVelocity() {
        return shooterEffort;
    }
    
}

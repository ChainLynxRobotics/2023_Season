package frc.robot.GenericPID.Implementations;

import java.util.ArrayList;

import frc.robot.GenericPID.Extensible.ControlStrategy;
import frc.robot.GenericPID.MotorControlProfile.ControlLevel;
import frc.robot.GenericPID.MotorControlProfile.UnknownControlStrategyException;

/**
 *Exert full force towards the setpoint, with estimated mass and dt known
 *so it won't overshoot in the time slice
 */
public class DtFullForce implements ControlStrategy {
    private double F;
    private double dt;
    private double m;
    public DtFullForce(double maxForce, double dt, double m) {
        F = maxForce;
        this.dt = dt;
        this.m = m;
    }
    public double calculate(double effectv, ControlLevel given, ControlLevel needed, double currv) throws UnknownControlStrategyException{
        if (given.ordinal() + 1 != needed.ordinal()) {
            throw new UnknownControlStrategyException("Conversion not compatible with this control strategy");
        }
        double betterF = F;
        if (Math.abs(currv - effectv) < dt * F / m) { // if needed Dv < calculated new dv / dt * Dt = calculated Dv
            betterF = Math.abs(currv - effectv) / dt * m; //new calculated Dv = needed Dv, new calculated dv / dt * Dt = needed Dv, new calculated F / m * Dt = needed Dv, nc F = nd Dv * m / Dt
        }
        if (currv < effectv) {
            return betterF;
        } else if (currv > effectv) {
            return -betterF;
        } else {
            return 0;
        }
    }
}

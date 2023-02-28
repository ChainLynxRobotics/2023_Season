package frc.robot.GenericPID.Implementations;

import java.util.ArrayList;

import frc.robot.GenericPID.ControlStrategy;
import frc.robot.GenericPID.MotorControlProfile.ControlLevel;
import frc.robot.GenericPID.MotorControlProfile.UnknownControlStrategyException;

public class DtFullForce implements ControlStrategy {
    private double F;
    private double dt;
    private double m;
    public DtFullForce(double maxForce, double dt, double m) {
        F = maxForce;
        this.dt = dt;
    }
    public double calculate(double effectv, ControlLevel given, ControlLevel needed, double currv) throws UnknownControlStrategyException{
        if (given.ordinal() + 1 != needed.ordinal()) {
            throw new UnknownControlStrategyException("Conversion not compatible with this control strategy");
        }
        double betterF = F;
        if (Math.abs(currv - effectv) < dt * F / m) {
            betterF = Math.abs(currv - effectv) / dt * m;
        }
        if (effectv < currv) {
            return betterF;
        } else if (effectv > currv) {
            return -betterF;
        } else {
            return 0;
        }
    }
}

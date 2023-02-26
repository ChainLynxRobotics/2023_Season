package frc.robot.GenericPID.Implementations;

import java.util.ArrayList;

import frc.robot.GenericPID.ControlStrategy;
import frc.robot.GenericPID.ControlEffectProfile.ControlLevel;
import frc.robot.GenericPID.ControlEffectProfile.UnknownControlStrategyException;

public class FullForce implements ControlStrategy {
    private double F;
    public FullForce(double maxForce) {
        F = maxForce;
    }
    public double determine(double effectv, ControlLevel given, ControlLevel needed, ArrayList<Double> currV_maxForce) throws UnknownControlStrategyException{
        double currv = currV_maxForce.get(0).doubleValue();
        double maxforce = currV_maxForce.get(1).doubleValue();
        if (given.ordinal() + 1 != needed.ordinal()) {
            throw new UnknownControlStrategyException("Conversion not compatible with this control strategy");
        }
        if (effectv < currv) {
            return currV_maxForce.get(1);
        } else if (effectv > currv) {
            return -maxforce;
        } else {
            return 0;
        }
    }
}

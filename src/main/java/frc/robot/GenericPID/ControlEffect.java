package frc.robot.GenericPID;

import java.util.ArrayList;

import frc.robot.GenericPID.ControlEffectProfile.ControlLevel;
import frc.robot.GenericPID.ControlEffectProfile.UnknownControlStrategyException;

public class ControlEffect {
    //a class that holds a control effect with the adapter

    private ControlEffectProfile c;
    private double effect;

    public Double motorOutput(ControlLevel motorEffect, ArrayList<Double> contextInfo) {
        try {
            return c.determine(effect, motorEffect, contextInfo);
        }
        catch (UnknownControlStrategyException e) {
            return null;
        }
    }
}
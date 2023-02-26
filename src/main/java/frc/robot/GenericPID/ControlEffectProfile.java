package frc.robot.GenericPID;

import java.util.ArrayList;

public class ControlEffectProfile {
    private ControlLevel controlEffect = ControlLevel.VELOCITY;
    private ControlStrategy converter;
    public ControlEffectProfile(ControlLevel controlEffect, ControlStrategy adapter) {
        this.controlEffect = controlEffect;
        converter = adapter;
    }
    public static class ControlLevelException extends RuntimeException { 
        public ControlLevelException(String errorMessage) {
            super(errorMessage);
        }
    }
    public static class UnknownControlStrategyException extends RuntimeException { 
        public UnknownControlStrategyException(String errorMessage) {
            super(errorMessage);
        }
    }
    public double determine(double effect, ControlLevel needed, ArrayList<Double> contextInfo) throws UnknownControlStrategyException {
        return converter.determine(effect, this.controlEffect, needed, contextInfo);
    }
    
    public static enum ControlLevel {
        ABSERK,
        ABSELERATION,
        ABSITY,
        ABSEMENT,
        POSITION,
        VELOCITY,
        ACCELERATION,
        JERK,
        SNAP
    }
}

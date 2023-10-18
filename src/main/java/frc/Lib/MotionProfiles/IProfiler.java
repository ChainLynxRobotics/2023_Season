package frc.Lib.MotionProfiles;

public interface IProfiler {

    public interface Config {

        public boolean atConfig();
    }

    public double calculate(double time);

    public boolean setpointReached(double curPos, double setpoint, double error);

}

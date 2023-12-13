package frc.MotionProfiles;

public interface IProfiler {

    public interface Config {

        public boolean atConfig(double curPos, double curVel);
    }

    public double calculate(double time);

}

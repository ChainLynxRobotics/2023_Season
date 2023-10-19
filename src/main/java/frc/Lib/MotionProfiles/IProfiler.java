package frc.Lib.MotionProfiles;

public interface IProfiler {

    public interface Config {

        public boolean atConfig();
    }

    public double calculate(double time);

}

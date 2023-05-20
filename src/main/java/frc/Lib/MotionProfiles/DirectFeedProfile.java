package frc.Lib.MotionProfiles;


public class DirectFeedProfile implements IProfiler {
    private double[] velocities;
    private double dt;

    public DirectFeedProfile(double[] velocities, double timestep) {
        this.velocities = velocities;
        dt = timestep;
    }

    @Override
    public double calculate(double t) {
        for (int i = 0; i < velocities.length; i++) {
            return velocities[i];
        }
        throw new UnsupportedOperationException("Unimplemented method 'calculate'");
    }

}

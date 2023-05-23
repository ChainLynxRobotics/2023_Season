package frc.Lib.MotionProfiles;


public class DirectFeedProfile implements IProfiler {
    private double[] velocities;
    private double dt;
    private double cur;

    public DirectFeedProfile(double[] velocities, double timestep) {
        this.velocities = velocities;
        dt = timestep;
    }

    @Override
    public double calculate(double t) {
        for (int i = 0; i < velocities.length;) {
            double initTime = System.currentTimeMillis();
            while (!setpointReached(cur, velocities[i], 0.5) && System.currentTimeMillis() - initTime < dt) {
                return velocities[i];
            }
        }
        return 0;
    }

    @Override
    public boolean setpointReached(double cur, double setpoint, double error) {
       if ((cur >= setpoint - error) && (cur <= setpoint + error)) {
        return true;
       }
       return false;
    }

}

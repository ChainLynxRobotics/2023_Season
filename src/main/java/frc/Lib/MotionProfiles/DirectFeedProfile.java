package frc.Lib.MotionProfiles;

public class DirectFeedProfile implements IProfiler {
    private double[] setpoints;
    private double dt;
    private double cur;
    private double maxVel;
    private double maxAccel;
    private DirectConfig initState;
    private DirectConfig finalState;

    public DirectFeedProfile(double maxVel, double maxAccel, DirectConfig initState, DirectConfig finalState, double timestep) {
        dt = timestep;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.initState = initState;
        this.finalState = finalState;
    }

    public class DirectConfig implements Config<DirectConfig> {

        public double position;
        public double velocity;

        public DirectConfig(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        @Override
        public boolean atConfig(DirectConfig config) {
            if (this.position == config.position && this.velocity == config.velocity) {
                return true;
            } else {
                return false;
            }
        } 
    }

    @Override
    public double calculate(double t) {
        for (int i = 0; i < setpoints.length;) {
            double initTime = System.currentTimeMillis();
            while (!setpointReached(cur, setpoints[i], 0.5) && System.currentTimeMillis() - initTime < dt) {
                return setpoints[i];
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

    public void setVelocityArray(double[] setpoints) {
        this.setpoints = setpoints;
    }

    public double[] sampleAlongProfile() {
        int numSamples = (int) Math.floor((finalState.position-initState.position)/dt)+1;
        double totalTime = (finalState.position-initState.position)/maxVel;
        double fullSpeedDist = finalState.position-initState.position - maxAccel*totalTime*totalTime;
        double[] samples = new double[numSamples];

        for (int i = 0; i < numSamples; i++) {
            double curTime = i*dt;
            if (curTime < maxVel/maxAccel) {
                samples[i] = maxAccel*curTime;
            } else if (curTime < maxVel/maxAccel+fullSpeedDist/maxVel) {
                samples[i] = maxVel;
            } else {
                samples[i] = maxVel - (totalTime-curTime)*maxAccel;
            }
            
        }

        return samples;
    }

}

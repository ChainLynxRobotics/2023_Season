package frc.Lib.MotionProfiles;

public class DirectFeedProfile implements IProfiler {
    private double[] setpoints;
    private double dt;
    private double curVel;
    private double maxVel;
    private double maxAccel;
    private DirectConfig initState;
    private DirectConfig finalState;
    private int stateCounter;

    public DirectFeedProfile(double maxVel, double maxAccel, DirectConfig initState, DirectConfig finalState, double timestep) {
        dt = timestep;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.initState = initState;
        this.finalState = finalState;
        this.curVel = 0;
        this.stateCounter = 0;
    }

    public class DirectConfig implements Config {

        public double position; //rotations
        public double velocity; //rpm

        public DirectConfig(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        @Override
        public boolean atConfig() {
            if (this.position == this.position && this.velocity == this.velocity) {
                return true;
            } else {
                return false;
            }
        } 
    }

    //should always be called before calculate
    public void setCurVel(double vel) {
        this.curVel = vel;
    }

    @Override
    public double calculate() {
        double initTime = System.currentTimeMillis();
        //if the current velocity setpoint still hasn't been reached, keep returning it
        if (!setpointReached(curVel, setpoints[stateCounter], 0.5) && System.currentTimeMillis() - initTime < dt) {
            return setpoints[stateCounter];
        }
        stateCounter++;
        return setpoints[stateCounter];
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
        int numSamples = (int) Math.floor((finalState.position - initState.position)/dt) + 1;
        double totalTime = ((finalState.position - initState.position) + 0.5*Math.pow(initState.velocity,2)/maxVel)/maxVel; //last term accounts for profile truncation if starting at nonzero initial velocity
        double fullSpeedDist = (finalState.position - initState.position) - Math.pow(maxVel, 2)/maxAccel + 0.5*(maxVel-initState.velocity)*initState.velocity/maxAccel;
        double rampUpTime = (maxVel - initState.velocity)/maxAccel;
        double[] samples = new double[numSamples];

        for (int i = 0; i < numSamples; i++) {
            double curTime = i*dt;
            if (curTime < rampUpTime) {
                samples[i] = maxAccel*curTime;
            } else if (curTime < rampUpTime + fullSpeedDist/maxVel) {
                samples[i] = maxVel;
            } else {
                samples[i] = maxVel - (totalTime-curTime)*maxAccel;
            }
            
        }

        return samples;
    }

    public boolean isProfileFinished() {
        if (stateCounter == setpoints.length - 1) {
            return true;
        }
        return false;
    }

}

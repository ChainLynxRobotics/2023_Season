package frc.Lib.MotionProfiles;

//without profile truncation
public class DirectFeedProfile implements IProfiler {
    private double[] setpoints;
    private double dt;
    private double maxVel;
    private double maxAccel;
    private DirectConfig initState;
    private DirectConfig finalState;
    private int stateCounter;

    private int numSamples; //number of velocity samples taken along the profile
    private double totalTime; 
    private double rampUpTime; //time taken to reach maximum velocity
    private double fullSpeedDist;


    public DirectFeedProfile(double maxVel, double maxAccel, DirectConfig initState, DirectConfig finalState, double timestep) {
        dt = timestep;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.initState = initState;
        this.finalState = finalState;
        this.stateCounter = 0;

        this.numSamples = (int) Math.floor((this.finalState.position - this.initState.position)/dt) + 1;

        //area of trapezoid = position difference = maxVel*totalTime
        this.totalTime = (this.finalState.position - this.initState.position)/maxVel;
        //this.totalTime = ((finalState.position - initState.position) + 0.5*Math.pow(initState.velocity,2)/maxVel)/maxVel; //last term accounts for profile truncation if starting at nonzero initial velocity

        this.rampUpTime = maxVel/maxAccel;
        //this.rampUpTime = (maxVel - this.initState.velocity)/maxAccel;

        //area of trapezoid - area ramp up + ramp down sections
        this.fullSpeedDist = (this.finalState.position - this.initState.position) - this.rampUpTime*maxVel;
        //this.fullSpeedDist = (finalState.position - initState.position) - Math.pow(maxVel, 2)/maxAccel + 0.5*(maxVel-initState.velocity)*initState.velocity/maxAccel;

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

    @Override
    public double calculate(double curTime) {
        //current setpoint is the velocity at the lowest time greater than the current time
        int setpointLoc = (int) Math.ceil(curTime*numSamples/totalTime);
        stateCounter = setpointLoc;
        return setpoints[setpointLoc];
    }

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
        double[] samples = new double[numSamples];

        for (int i = 0; i < numSamples; i++) {
            double curTime = i*dt;
            if (curTime < rampUpTime) {
                samples[i] = maxAccel*curTime; //velocity increasing
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

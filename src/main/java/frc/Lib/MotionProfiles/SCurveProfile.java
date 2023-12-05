package frc.Lib.MotionProfiles;

//import org.apache.commons.lang3.Range;

import frc.robot.Constants;

public class SCurveProfile extends TrapezoidalProfile {
    private double t1;
    private double t2;
    private double t3;
    private double t4;

    private SConfig initState;
    private SConfig finalState;

    private final double avAccel;
    private final double jerk;

    public static class SConfig extends TrapezoidalProfile.Config {
        private double accel;

        public SConfig(double position, double velocity, double accel) {
            super(position, velocity);
            this.accel = accel;
        }

        public boolean atConfig(SConfig config) {
            if (this.position == config.position && this.velocity == config.velocity && this.accel == accel) {
                return true;
            } else {
                return false;
            }
        }
        
    }

    public SCurveProfile(double maxAccel, double maxVel, SConfig initState, Config finalState) {
        super(maxAccel, maxVel, initState, finalState);

        if (initState.position > finalState.position) {
            direction = -1;
        }

        this.initState = adjustProfile(initState);

        avAccel = Math.sqrt(maxVel/2);
        jerk = maxAccel*avAccel/((maxVel-initState.velocity)*(maxAccel-avAccel));

        t1 = maxAccel/jerk;
        t2 = maxVel/avAccel - maxAccel/jerk;
        t3 = maxVel/avAccel;
        t4 = (finalState.position - initState.position)/maxVel-2*maxVel/avAccel;
    }

    public SCurveProfile(double maxAccel, double maxVel, Config finalState) {
        this(maxAccel, maxVel, new SConfig(0, 0, 0), finalState);
    }

    //calculates Config of SCurve motion profile (7 profile states)
    public SConfig calculate(double t) {
        SConfig config = new SConfig(initState.position, initState.velocity, initState.accel);
        double checkpointVel = 0;
        double checkpointPos = 0;
        System.out.println("time is: " + t);

        if (t < t1) {
            config.accel = jerk*t;
            config.velocity = jerk*t*t/2;
            config.position = jerk*t*t*t/6;
        } else if (t < t2) {
            config.accel = maxAccel;
            config.velocity = jerk*t1*t1/2 + maxAccel*(t-t1);
            config.position = jerk*t1*t1*(t1-t)/2 + maxAccel*(t-t1)*(t-t1)/2+jerk*t*t*t/6;
        } else if (t < t3) {
            config.accel = maxAccel - jerk*(t-t2);
            config.velocity = maxVel - jerk*(t-t3)*(t-t3)/2;
            config.position = maxVel*maxVel/(2*avAccel)+maxVel*(t-t3) - jerk*Math.pow(t-t3,3)/6;
        } else if (t < t4) {
            config.accel = 0;
            config.velocity = maxVel;
            config.position = maxVel*maxVel/(2*avAccel)+maxVel*t3 - jerk*t3*t3*t3/6 + maxVel*(t4-t);
            isInRange(t4, checkpointPos, config.position, t);
        } else if (t < t4 + t3 - t2) {
            config.accel = jerk*(t-t4);
            config.velocity = maxVel - jerk*(t-t4)*(t-t4)/2;
            config.position = checkpointPos + maxVel*maxVel/(2*avAccel)+maxVel*(t - (t4 + t3 - t2)) - jerk*Math.pow(t-(t4 + t3 - t2), 3)/6;
            isInRange(t4, checkpointPos, config.velocity, t);
            isInRange(t4, checkpointPos, config.position, t);
        } else if (t < t4 + t3 - t1) {
            config.accel = -maxAccel;
            config.velocity = checkpointVel - maxAccel*(t - (t4 + t3 - t2));
            config.position = checkpointPos - checkpointVel*(t - (t4 + t3 - t2)) - maxAccel*Math.pow((t - (t4 + t3 - t2)), 2)/2;
            isInRange(t4, checkpointPos, config.velocity, t);
            isInRange(t4, checkpointPos, config.position, t);
        } else if (t <= t4 + t3) {
            config.accel = -maxAccel + jerk*(t-(t4 + t3 - t1));
            config.velocity = checkpointVel - maxAccel*(t-(t4 + t3 - t1)) - jerk*Math.pow(t-(t4 + t3 - t1),2)/2;
            config.position = checkpointPos + checkpointVel*(t-(t4 + t3 - t1)) - maxAccel*Math.pow(t-(t4 + t3 - t1),2)/2 - jerk*Math.pow(t-(t4 + t3 - t1),3)/6;
        } else {
            return finalState;
        }

        System.out.println("config velocity is: " + config.velocity + " and position is: " + config.position);

        return config;
    }

    public void isInRange(double curCheckpoint, double checkpoint, double curr, double t) {
        /*Range<Double> range = Range.between(curCheckpoint - Constants.GLOBAL_TIMESTEP, curCheckpoint + Constants.GLOBAL_TIMESTEP);
    
        if (range.contains(t)) {
            checkpoint = curr;
        }*/
    }

    

    protected SConfig adjustProfile(SConfig config) {
        SConfig current = new SConfig(config.position, config.velocity, config.accel);
        current.position *= direction;
        current.velocity *= direction;
        current.accel *= direction;
        return current;
    }

    
}

package frc.Lib.MotionProfiles;

public class TrapezoidalProfile {
    protected int direction;
    protected double maxAccel;
    protected double maxVel;
    private double initAccelTime;
    protected double fullSpeedTime;
    private double endAccelTime;

    private Config initState;
    private Config finalState;

    public static class Config {
        public double position;
        public double velocity;

        public Config(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        public boolean atConfig(Config config) {
            if (this.position == config.position && this.velocity == config.velocity) {
                return true;
            } else {
                return false;
            }
        }
    }

    public TrapezoidalProfile(double maxAccel, double maxVel, Config initState, Config finalState) {
        direction = 1;
        if (initState.position > finalState.position) {
            direction = -1;
        }
        this.initState = adjustProfile(initState);
        this.finalState = adjustProfile(finalState);
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;

        if (this.initState.velocity  > maxVel) {
            this.initState.velocity = maxVel;
        }
       
        //truncation calculations (if motion profile starts at 0)
        double rampUpTime = initState.velocity/maxAccel;
        double beginRampDist = rampUpTime*rampUpTime*initState.velocity/2;

        double rampDownTime = finalState.velocity/maxAccel;
        double endRampDist = rampUpTime*rampUpTime*finalState.velocity/2;

        double totalDist = beginRampDist + (finalState.position - initState.position) + endRampDist;
        double accelTime = maxVel/maxAccel;
        double fullSpeedDist = totalDist - maxAccel*accelTime*accelTime;

        if (fullSpeedDist < 0) { 
            accelTime = Math.sqrt(totalDist/maxAccel);
            fullSpeedDist = 0;
        }

        initAccelTime = accelTime/2-rampUpTime;
        fullSpeedTime = initAccelTime + fullSpeedDist/maxVel;
        endAccelTime = fullSpeedTime + accelTime/2 - rampDownTime;
    }

    public TrapezoidalProfile(double maxAccel, double maxVel, Config finalState) {
        this(maxAccel, maxVel, new Config(0,0), finalState);
    }

    //calculate config for state at a time t (3 possible states -- + accel, 0 accel + max vel, - accel)
    public Config calculate(double t) {
        Config current = new Config(initState.position, initState.velocity);

        if (t < initAccelTime) {
            current.velocity = initState.velocity + maxAccel*t;
            current.position = initState.velocity*t + maxAccel*t*t/2;
        } else if (t < fullSpeedTime) {
            current.velocity = maxVel;
            current.position = initAccelTime*maxVel/2 + maxVel*(t-initAccelTime);
        } else if (t <= endAccelTime) {
            current.velocity = maxVel - maxAccel*(t-fullSpeedTime);
            current.position = initAccelTime*maxVel/2 + maxVel*(fullSpeedTime-initAccelTime)+maxVel*(t-fullSpeedTime)-maxAccel*Math.pow(t-fullSpeedTime,2)/2;
        } else {
            return finalState;
        }

        return adjustProfile(current);
    }

    public double getTotalProfileTime() {
        return endAccelTime;
    }

    protected Config adjustProfile(Config config) {
        Config current = new Config(config.position, config.velocity);
        current.position *= direction;
        current.velocity *= direction;
        return current;
    }


}

package frc.MotionProfiles;

public class TrapezoidalProfile {
    private double maxAccel;
    private double maxVel;

    private Config initState;
    private Config finalState;

    public static class Config {
        private double position;
        private double velocity;

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
        int scale = 1;
        if (initState.position > finalState.position) {
            scale = -1;
        }
        this.finalState.velocity = scale*finalState.velocity;
        this.finalState.position = scale*finalState.position;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.initState = initState;

        if (this.initState.velocity  > maxVel) {
            this.initState.velocity = maxVel;
        }
    }

    public TrapezoidalProfile(double maxAccel, double maxVel, Config finalState) {
        this(maxAccel, maxVel, new Config(0,0), finalState);
    }

    //truncation calculations

    //calculate config for state at a time t

    
}

package frc.Lib.MotionProfiles;

public class SCurveProfile extends TrapezoidalProfile {

    public SCurveProfile(double maxAccel, double maxVel, Config finalState) {
        super(maxAccel, maxVel, finalState);

        class SCurveConfig extends TrapezoidalProfile.Config {
            private double accel;

            public SCurveConfig(double position, double velocity, double accel) {
                super(position, velocity);
                this.accel = accel;
            }

            @Override
            public boolean atConfig(Config config) {
                if (this.position == config.position && this.velocity == config.velocity && this.accel == accel) {
                    return true;
                } else {
                    return false;
                }
            }
            
        }
    }

    @Override
    public Config calculate(double t) {
        return null;
    }

    @Override
    protected Config adjustProfile(Config config) {
        return null;
    }

    
}

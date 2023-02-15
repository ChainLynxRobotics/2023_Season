package frc.robot.GenericPID;

public class GenericPIDMotorFollow {
    private GenericPath path;
    private GenericPID pid;
    public CatchUpFunction catchUpDt;
    private int timesBehind = 0;
    public double dt;
    private double lastEffect = 0;

    public interface CatchUpFunction {
        public double dt(double timeBehind, int timesBehind, double normalDt);
    }

    public class FullCatchUp implements CatchUpFunction {
        public double dt(double timeBehind, int timesBehind, double normalDt) {
            return timeBehind;
        }
    }

    public GenericPIDMotorFollow(GenericPath path, PIDConfig config, double dt) {
        this.path = path;
        this.pid = new GenericPID(config);
        this.catchUpDt = new FullCatchUp();
        this.dt = dt;
    }
    public double pollControlEffect(double x, double t) {
        if (t < pid.t()) {
            timesBehind = 0;
            return lastEffect;
        }

        if (t > pid.t() + dt) {
            double timeBehind = t - pid.t() - dt;
            timesBehind++;
            double dt = catchUpDt.dt(timeBehind, timesBehind, this.dt);
            double effect = nextControlEffect(x, dt);
            lastEffect = effect;
            return effect;
        }

        if (t > pid.t()) {
            timesBehind = 0;
            double effect = nextControlEffect(x, dt);
            lastEffect = effect;
            return effect;
        }

        int no_this_wont_happen_because_math_and_java_is_stupid = 69420;
        return no_this_wont_happen_because_math_and_java_is_stupid;
    }
    public double nextControlEffect(double x, double dt) {
        return pid.controlEffect(path.y(pid.t()), x, dt);
    }
}
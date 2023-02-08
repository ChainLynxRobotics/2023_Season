public class GenericPID {
    //a basic pid controller class, that lets the motor handle tracking position, velocity, and max of these. 
    //for this functionality, extend this class (could override controlEffect)
    //the max effective target velocity should be implicitly in line with the tuning, and the motor should cap itself. This object is quite basic.
    private PIDConfig conf;
    private double t;
    private ApproximateDerivative dedt;
    private ApproximateIntegral E;
    public GenericPID(PIDConfig config, double maxMotorVelocity) {
        this.config = c;
    }
    public double t() {
        return t;
    }
    public double controlEffect(double target, double current, double dt) {
        double curre = target - current;
        double currdedt = dedt.nextDerivative(t, dt);
        double currE = E.next(e, dt);
        return conf.kP * e + conf.kI * currE + conf.kD * currdedt;
    }
}
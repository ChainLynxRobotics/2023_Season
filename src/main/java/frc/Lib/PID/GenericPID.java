package frc.Lib.PID;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;


public class GenericPID {
    //a basic pid controller class, that lets the motor handle tracking position, velocity, and max of these. 
    //for this functionality, extend this class (could override controlEffect)
    //the max effective target velocity should be implicitly in line with the tuning, and the motor should cap itself. This object is quite basic.
    //the motor should listen to pid after each control effect, and keep track of its own time value too (this can be a real life time value)
    //graph of how the pid time should update
    //t .          .          .
    //   |________| |________|
    //    motor i0   motor i1    (you handle)
    //  di0  di1        di2      (internal) 
    //   i0        i1       i2   (internal)
    //   p0        p1       p2   (internal)
    
    //todo: maybe genericPID Profile that can use any derivative sorta function? it is generic, but would be kinda useless

    private static final boolean debug = false;

    private PIDConfig conf;
    private double t;
    private ApproximateDerivative dedt;
    private ApproximateIntegral E;


    public GenericPID(PIDConfig config) {
        this.conf = config;
        this.t = 0;
        this.dedt = new ApproximateDerivative(0, 0);
        this.E = new ApproximateIntegral(0, 0);
    }

    //converts control effect into motor power
    public double normalize(double curr, double setpoint) {
        double rawOutput = posControlEffect(setpoint, curr, Constants.GLOBAL_TIMESTEP);
        return MathUtil.clamp(rawOutput, 0, 1);
    }

    public double firstEffect(double target, double current) {
        return firstEffect(target, current, 0); //literally dt doesnt matter, just a placebo
    }
    public double firstEffect(double target, double current, double dt) {
        double curre = target - current;
        dedt.reset(0, curre);
        double eff = conf.kP * curre; //no accumulation, unknown derivative
        if(debug)System.out.printf("first effect! %f curre! %f\n" , eff, curre);
        return eff;
    }
    public double posControlEffect(double target, double current, double dt) {
        double curre = target - current; //error term, in direction of target, positive means below
        //need positive P to make motor go that way
        double currdedt = dedt.nextDerivative(t + dt, curre); //derivative term, derivative of error in direction of target - means going towards
        //need positive D to make motor go less when the narrowing of error is going more [negative]
        //if the error is closing in too fast (too negative), then the motor will have less target velocity to slow down more
        E.next(curre, dt); //integral term, sum of error in direction of target, positive means has been going towards
        //draw integral as shading between target and current, target over current is positive
        //as more error is accumulated, the positive I will make the target velocity higher to speed up the motor as more pressure comes on
        double currE = E.val();
        double eff = conf.kP * curre + conf.kI * currE + conf.kD * currdedt;
        if(debug)System.out.printf("effect! %f curre! %f currE! %f currde %f\n" , eff, curre, currE, currdedt);
        return eff;
    }

    public double velControlEffect(double target, double current, double dt) {
        return conf.kP*(target-current) + conf.kD*dedt.nextDerivative(t + dt, target-current);
    }

    public void next_t(double dt) {
        t += dt;
    }

    public boolean synced_exact(double t) {
        return (t == this.t);
    }

    public boolean synced_range(double t, double range) { //range is usually equal to dt but depends on checking situation
        return (Math.abs(t - this.t) > range);     
    }

    public double t() {
        return t;
    }
}
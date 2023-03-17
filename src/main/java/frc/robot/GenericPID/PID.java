package frc.robot.GenericPID;

import java.awt.Color;

import frc.robot.GenericPID.ApproximationUtils.*;
import frc.robot.GenericPID.Testing.*;
/**
  *A basic pid controller class, that lets the motor handle tracking position, velocity, and max of these. 
  *For this functionality, extend this class (could override controlEffect).
  the max effective target velocity should be implicitly in line with the tuning, and the motor should cap itself. This object is quite basic.
  *The motor should listen to pid after each control effect, and keep track of its own time value too (this can be a real life time value)
  */
  public class PID {
      //graph of how the pid time should update
      // .          .          .
      //  |________| |________|
      //   motor i0   motor i1    (you handle)
      // di0  di1        di2      (internal) 
      //  i0        i1       i2   (internal)
      //  p0        p1       p2   (internal)
      
      //todo: maybe PID Profile that can use any derivative sorta function? it is generic, but would be kinda useless

    private static final boolean debug = Debug.debug_GenericPID;

    private PIDConfig conf;
    private double t;
    private ApproximateDerivative dedt;
    private ApproximateIntegral E;
    public static void main(String[] args) {
        test();
    }
    
    public PID(PIDConfig config) {
        this.conf = config;
        this.t = 0;
        this.dedt = new ApproximateDerivative(0, 0);
        this.E = new ApproximateIntegral(0, 0);
    }

    public static void test() {
        final double l = 3;
        PIDConfig c = new PIDConfig();
        c.kP = 1.0;//1.7;
        c.kI = 0.0;//0.0001;
        c.kD = 0.01;//0.06;
        PID p = new PID(c);
        //ArtificialMotor m = new ArtificialMotor(2, 0.1, 0, 2, 0, 0, 0);
        ArtificialMotor m2 = new ArtificialMotor.Builder().v0(2).build();
        ArtificialMotor m = new ArtificialMotor(1, 0.00, 0, 1);
        double dt = 0.005;
        double target = 5;//2;
        // Graph g = new Graph(new Graph.GraphConfig.Builder.y2(100).build());
        Graph.GraphConfig gc = new Graph.GraphConfig();
        gc.x1 = 0;
        gc.x2 = l;//20;
        gc.y1 = -1;//-3;
        gc.y2 = 7;//3;
        Graph g = new Graph(gc);
        g.addPlot(Color.BLUE);
        g.addPlot(Color.RED);
        g.addPlot(Color.GREEN);
        for (double ti = 0; ti < l; ti+=dt) {
            //first the control effect is updated (including with the derivative of currrent to last),
            //then the motor is directed towards this velocity, and the motor updates its position and known time.
            //then the pid updates its known time, should be in sync with the motor's known time
            //see graph atop this class
            
            //assume this will just be in sync, use reset if needed
            double ce = p.controlEffect(target, m.p(), dt);
            double f; //manual handling with PID, PIDMotorFollow includes this abstraction for you
            if (ce > m.v()) {
                f = m.maxF();
            } else if (ce < m.v()) {
                f = -m.maxF();
            } else {
                f = 0;
            }
            m.exert(f ,dt); 
            p.next_t(dt);
            assert p.synced_exact(m.t());
            if(debug)System.out.printf("t:%f p:%f v:%f\n", ti, m.p(), m.v());
            g.addPoint(ti, m.p(), false, 0);
            g.addPoint(ti, m.v(), false, 1);
        }
        g.init(1000,1000,"PID Test");
    }

    private double firstEffect(double target, double current) {
        return firstEffect(target, current, 0); //literally dt doesnt matter, just a placebo
    }
    private double firstEffect(double target, double current, double dt) {
        double curre = target - current;
        dedt.reset(0, curre);
        double eff = conf.kP * curre; //no accumulation, unknown derivative
        if(debug)System.out.printf("first effect! %f curre! %f\n" , eff, curre);
        return eff;
    }
    public double controlEffect(double target, double current, double dt) {
        //if we haven't evaluated a controleffect yet, we will use firstEffect
        // which ignores derivative and integral which are useless
        if (t == 0) {
            return firstEffect(target, current);
        }

        double curre = target - current; //error term, in direction of target, positive means below
        //need positive P to make motor go that way
        double currdedt = dedt.nextDerivative(t + dt, curre); //derivative term, derivative of error in direction of target - means going towards
        //need positive D to make motor go less when the narrowing of error is going more [negative]
        //if the error is closing in too fast (too negative), then the motor will have less target velocity to slow down more
        E.next(curre, dt); //integral term, sum of error in direction of target, positive means has been going towards
        //draw integral as shading between target and current, target over current is positive
        //as more error is accumulated, the positive I will make the target velocity higher to speed up the motor more as pressure comes on
        double currE = E.val();
        double eff = conf.kP * curre + conf.kI * currE + conf.kD * currdedt;
        if(debug)System.out.printf("effect! %f curre! %f currE! %f currde %f\n" , eff, curre, currE, currdedt);
        return eff;
    }
    public void next_t(double dt) {
        t += dt;
    }
    public boolean synced_exact(double t) { 
        //time for pid always starts at zero and should be relatively handled so it stays staggered around real time
        return (t == this.t);
    }
    public boolean synced_range(double t, double range) { //range is usually equal to dt but depends on checking situation
        return (Math.abs(t - this.t) > range);     
    }
    public double t() {
        return t;
    }
}
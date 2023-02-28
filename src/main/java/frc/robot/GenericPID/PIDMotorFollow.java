package frc.robot.GenericPID;

import frc.robot.GenericPID.Approximations.DoubleFunction;
import frc.robot.GenericPID.Implementations.FullCatchUp;
import frc.robot.GenericPID.Implementations.FullForce;
import frc.robot.GenericPID.Implementations.LinearSegment;
import frc.robot.GenericPID.MotorControlProfile.ControlLevel;
import frc.robot.GenericPID.Testing.ArtificialMotor;
import frc.robot.GenericPID.Testing.Graph;

import java.awt.Color;
import java.util.Random;

public class PIDMotorFollow {
    //an object which drives together a pid controller with a path and a controleffect profile for fully driving the motor.
    //todo: split into motor on top of lower level version connects path and pid??
    //todo: maybe make this more similar to frc code and compiles down?
    boolean debug = Debug.debug_MotorFollow;
    
    private Path path;
    private PID pid;
    private MotorControlProfile motor;

    public CatchUpFunction catchUpDt; //customize this if you want by just swapping its value with another CatchUpFunction
    private int timesBehind = 0;
    public double dt;
    
    private double lastEffect = 0;

    
    public static void main(String[] args) {
        test();
    }

    public PIDMotorFollow(Path path, PIDConfig config, MotorControlProfile motorprofile, double dt) {
        this.path = path;
        this.pid = new PID(config);
        this.catchUpDt = new FullCatchUp();
        this.dt = dt;
    }
    
    public static void test() {
        //a test and example for a pidmotorfollower

        //simulation for time variance in processing
        Random random = new Random();

        //make the path and put segments in it
        var p = new Path();
        // p.insertSegment(new LinearSegment(0,1,15,16));
        // p.insertSegment(new LinearSegment(15,16,30,1));
        p.insertSegment(new LinearSegment(0,0,10,0));
        p.insertSegment(new LinearSegment(10,5,30,5));

        //simulation for the motor
        double maxf = 100;
        var m = new ArtificialMotor(1, 0.01, 0, maxf);
        var mc = new MotorControlProfile(ControlLevel.VELOCITY, ControlLevel.ACCELERATION, new FullForce(maxf));

        //target control effect grain
        double dt = 0.005;

        //follower class, takes a path, pid config, and target dt
        double kP = 1;
        double kI = 0.00;
        double kD = 0.01; 
        var g = new PIDMotorFollow(p, new PIDConfig(kP,kI,kD), mc, dt);

        //realtime simulation variable
        double t = 0;
        
        //graph for testing
        var gc = new Graph.GraphConfig();
        gc.x2 = 31;
        gc.x1 = 0;
        gc.y1 = -1;
        gc.y2 = 5;
        Graph G = new Graph(gc);
        G.addPlot(Color.RED);

        //go for thirty fake seconds of following the control effect at target intervals, and graph it
        while (t < 30) {
            m.exert(g.motorOutputNow(m.p(), t), dt);
            t += random.nextDouble() / 1000;
            G.addPoint(t, m.p(), 0);
        }

        //plot path for comparison
        G.addPlot(Color.BLUE);
        DoubleFunction F = new DoubleFunction() {
            public double eval(double x) {
                Double ret = p.y(x);
                if (ret == null) {
                    return 0;
                } else {
                    return ret;
                }
            }
        };
        G.plot(0, 30, F, dt, 1);

        //output graph
        G.init(1000,1000, "folow");
    }

    public double motorOutputNow(double x, double t) {
        //Decides whether or not to redetermine control effect
        //Converts to motor output using the controleffectprofile's adapater
        if(debug) System.out.printf("Realtime: %f PID time: %f\n", t, pid.t());
        if (t < pid.t()) {
            if(debug) System.out.println("PID time already covered!");
            timesBehind = 0;
            return lastEffect;
        }

        if (t > pid.t() + dt) {
            double timeBehind = t - pid.t() - dt;
            timesBehind++;
            double newdt = catchUpDt.dt(timeBehind, timesBehind, dt);
            if(debug) System.out.printf("PID time behind, using catchupfunction to catchup! timebehind %f timesbehind %d dt %f to catch up!\n", timeBehind, timesBehind, newdt);
            double effect = nextControlEffect(x, newdt);
            lastEffect = effect;
            return motor.mEffect(effect, effect);
        }

        if (t > pid.t()) {
            if(debug) System.out.println("PID time on track!");
            timesBehind = 0;
            double effect = nextControlEffect(x, dt);
            lastEffect = effect;
            return effect;
        }
        
        int no_this_wont_happen_if_no_data_race = 69420;
        return no_this_wont_happen_if_no_data_race;
    }

    public double nextControlEffect(double x, double dt) {
        //it's okay for derivative to track change in path too because it just works; 
        //will start dampening when slope is greater than n instead of 0
        double ret = pid.controlEffect(path.y(pid.t()), x, dt);
        pid.next_t(dt);
        return ret;
    }
}
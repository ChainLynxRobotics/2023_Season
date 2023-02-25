package frc.robot.GenericPID;

import frc.robot.GenericPID.Testing.ArtificialMotor;
import frc.robot.GenericPID.Testing.Graph;

import java.awt.Color;
import java.util.Random;

public class GenericPIDMotorFollow {
    private GenericPath path;
    private GenericPID pid;
    public CatchUpFunction catchUpDt;
    private int timesBehind = 0;
    public double dt;
    
    private double lastEffect = 0;
    
    public static void main(String[] args) {
        test();
    }
    
    public static void test() {
        Random random = new Random();
        var p = new GenericPath();
        p.insertSegment(new LinearSegment(0,1,1,2));
        p.insertSegment(new LinearSegment(1,2,2,1));
        var m = new ArtificialMotor(1, 0.01, 0, 1);
        double dt = 0.005;
        var g = new GenericPIDMotorFollow(p, new PIDConfig(1,0.1,0.01), dt);
        double t = 0;
        var gc = new Graph.GraphConfig();
        gc.x2 = 31;
        gc.x1 = 0;
        gc.y1 = -1;
        gc.y2 = 5;
        Graph G = new Graph(gc);
        G.addPlot(Color.BLUE);
        while (t < 30) {
            m.sustain(g.pollControlEffect(m.p(), t), dt);
            t += random.nextDouble() / 100;
            G.addPoint(t, m.p(), 0);
        }
        G.init(1000,1000, "folow");
    }
    
    public interface CatchUpFunction {
        public double dt(double timeBehind, int timesBehind, double normalDt);
    }

    public class FullCatchUp implements CatchUpFunction {
        public double dt(double timeBehind, int timesBehind, double normalDt) {
            return timeBehind;
        }
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
    public GenericPIDMotorFollow(GenericPath path, PIDConfig config, double dt) {
        this.path = path;
        this.pid = new GenericPID(config);
        this.catchUpDt = new FullCatchUp();
        this.dt = dt;
    }
}
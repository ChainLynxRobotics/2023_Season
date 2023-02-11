package frc.robot.GenericPID.Testing;
import frc.robot.GenericPID.Approximations.*;


import java.lang.Math;
import java.awt.Color;

public class ArtificialMotor {
    //An artificial motor class used for simulation
    //the motor can absolutely control acceleration, and therefore can resultingly control velocity and position.
    //therefore, the motor has behavior to target the velocity (because there's no overshoot)
    //position should be controlled with a separate PID controller, because that's what it's for
    private final double mass; //SI, kg
    private final double kdrag; //SI, unitless
    private final double kkdrag; //SI, units?
    private final double force; //SI, N //TODO: should make a max accel too, this is max force? idk, this makes sense as is kinda, unless constraints are needed for external parts
    //or make it so this is max force and you can use less, control it yourself? irdk, tbf, this is just a simulation

    //the artificial motor is simulated with tangential physics, not rotational
    //So anytime the motor has a force applied, kdrag * v is subtracted, and 
    //Acceleration is calculated by mass

    private ApproximateIntegral velocity = new ApproximateIntegral(0, 0); //SI, integrated; m/s
    private ApproximateIntegral position = new ApproximateIntegral(0.0, 0.0); //SI, integrated; m
    private double t;
    
    public static void test() {
        final int n = 6;
        ArtificialMotor[] m = new ArtificialMotor[n];
        Color[] c = new Color[]{Color.BLUE, Color.RED, Color.GREEN, Color.BLUE.darker(), Color.RED.darker(), Color.GREEN.darker()};
        for (int i = 0; i < n/2; i++) {
            m[i] = new ArtificialMotor(1, i * 0.2, 0, 0.5);
        }
        for (int i = n/2; i < n; i++) {
            m[i] = new ArtificialMotor(1, 0, (i-n/2) * 0.2, 0.5);
        }
        Graph g = new Graph(new Graph.GraphConfig());
        g.init(1000,1000, "Motor to 1 m/s");

        for(int i = 0; i < n; i++) {
            g.addPlot(c[i]);
            double dt = 0.01;
            while (m[i].t() < 10) {
                m[i].go(dt);
                g.addPoint(m[i].t(), m[i].p(), i);
            }
            // while (m[i].t() < 10) {
            //     m[i].direct(1, dt);
            //     g.addPoint(m[i].t(), m[i].p(), i);
            // }
            // while (m[i].t() < 15) {
            //     m[i].slow(dt);
            //     g.addPoint(m[i].t(), m[i].p(), i);
            // }
        }
    }
    public static void main(String[] args) {
        test();
    }

    public void go(double dt) {
        velocity.next( (force - kdrag * velocity.val() - kkdrag * velocity.val() * Math.abs(velocity.val())) / mass, dt); //subtracts kdrag times velocity and kkdrag times directional velocity squared, always drags in right direction
        position.next(velocity.val(), dt);
        t += dt;
    }
    public void slow(double dt) {
        velocity.next( (-force - kdrag * velocity.val() - kkdrag * velocity.val() * Math.abs(velocity.val())) / mass, dt);
        position.next(velocity.val(), dt);
        t += dt;
    }
    public void active(double dt) {
        velocity.next( (0 - kdrag * velocity.val() - kkdrag * velocity.val() * Math.abs(velocity.val())) / mass, dt);
        position.next(velocity.val(), dt);
        t += dt;
    }

    public void exert(double F, double dt) {
        if (Math.abs(F) > force) { //cap it
            F = force * Math.abs(F) / F;
        }
        velocity.next( (0 - kdrag * velocity.val() - kkdrag * velocity.val() * Math.abs(velocity.val())) / mass, dt);
        position.next(velocity.val(), dt);
        t += dt;
    }
    
    public void direct(double v, double dt) {
        if (velocity.val() < v) {
            go(dt);
        }
        else if (velocity.val() > v) {
            slow(dt);
        } 
        else {
            active(dt);
        }
    }

    public void run(double v, double dt, double tlast) {
        while(t < tlast) {
            direct(v, dt);
        }
    }

    public double v() {
        return velocity.val();
    }
    public double p() {
        return position.val();
    }
    public double t() {
        return t;
    }
    public ArtificialMotor() {
        this.mass = 2;
        this.kdrag = 5;
        this.kkdrag = 5;
        this.force = 1;
    }
    public ArtificialMotor(double mass, double kdrag, double kkdrag, double force) {
        this.mass = mass;
        this.kdrag = kdrag;
        this.kkdrag = kkdrag;
        this.force = force;
    }
}
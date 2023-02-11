package frc.robot.GenericPID.Testing;
// package frc.robot.GenericPID.Approximations.Testing;
// import  frc.robot.GenericPID.Approximations.Testing.Graph;
// import  frc.robot.GenericPID.Approximations.Testing.Graph;
// import frc.robot.GenericPID.Approximations.ApproximateDerivative;
import frc.robot.GenericPID.Approximations.*;
// import frc.robot.GenericPID.Approximations.*;

// package frc.robot.GenericPID.Testing;
// import frc.robot.GenericPID.Approximations;
// import frc.robot.GenericPID.Approximations.*;
// import frc.robot.GenericPID.Approximations.ApproximateIntegral;
// import frc.robot.GenericPID.approximations.ApproximateIntegral;
// import frc.robot.GenericPID.Approximations.approximations.ApproximateIntegral;
// import frc.robot.GenericPID.Approximations.*;
// import frc.robot.GenericPID.Approximations.Graph;
// import frc.robot.GenericPID.Approximations.ApproximateDerivative;
// import frc.robot.GenericPID.Approximations.ApproximateIntegral;
// import frc.robot.GenericPID.Approximations.DoubleFunction;
// import frc.robot.GenericPID.Approximations.Zero;

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
    private ApproximateDerivative d = new ApproximateDerivative(); //dingusss
    private Zero z = new Zero(); //dingusss2
    private double t;

    public static void main(String[] args) {
        ArtificialMotor m = new ArtificialMotor();
        Graph g = new Graph(new Graph.GraphConfig());
        g.init(500,500, "Motor to 1 m/s", Color.BLUE);	

        while (m.t() < 5) {
            m.direct(1, 0.01);
            g.addPoint(m.t(), m.p());
        }

    }

    public void go(double dt) {
        double netdrag = kdrag * velocity + kkdrag * velocity * Math.abs(velocity); //always matches the sign of velocity, forces are added
        if (Math.abs(netdrag) > Math.abs(force)) {
            netdrag = force;
        }
        velocity.next( (force - kdrag * velocity - kdrag * velocity * Math.abs(velocity)) / mass, dt);
        position.next(velocity, dt);
        t += dt;
    }
    public void slow(double dt) {
        velocity.next(-(force - kdrag * velocity) / mass, dt);
        position.next(velocity, dt);
        t += dt;
    }
    public void active(double dt) {
        velocity.next(0, dt);
        position.next(velocity, dt);
        t += dt;
    }
    
    public void direct(double v, double dt) {
        if (velocity < v) {
            go(dt);
        }
        else if (velocity > v) {
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
        return velocity;
    }
    public double p() {
        return position;
    }
    public double t() {
        return t;
    }
    public ArtificialMotor() {
        this.mass = 2;
        this.kdrag = 0.1;
        this.kkdrag = 0.1;
        this.force = 1;
    }
    public ArtificialMotor(double mass, double kdrag, double kkdrag, double force) {
        this.mass = mass;
        this.kdrag = kdrag;
        this.kkdrag = kkdrag;
        this.force = force;
    }
}
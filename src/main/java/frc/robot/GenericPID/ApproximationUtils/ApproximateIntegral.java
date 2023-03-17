package frc.robot.GenericPID.ApproximationUtils;
// package Approximations;

import frc.robot.GenericPID.Extensible.DoubleFunction;
import frc.robot.GenericPID.Implementations.Zero;

public class ApproximateIntegral {
    private double x1;
    private double x;
    private double y1;
    private double Y;
    private DoubleFunction f;


    public ApproximateIntegral(double x, double y) {
        this.reset(x, y);
        f = new Zero();
    }
    
    public ApproximateIntegral(DoubleFunction f, double x, double y) {
        this.reset(f, x, y);
    }

    public double x() {
        return x;
    }
    public double x1() {
        return x1;
    }
    public double y1() {
        return y1;
    }
    public double val() {
        return Y;
    }
    //not requiring a function for this so there can be an alternative of the next method
    //unfortunately integrals can't exactly be staic like derivatives so this is how it looks
    public void reset(double x, double y) {
        this.x = x;
        this.x1 = x;
        this.y1 = y;
        this.Y = y;
    }
    public void reset(DoubleFunction f, double x, double y) {
        this.x = x;
        this.x1 = x;
        this.y1 = y;
        this.Y = y;
        this.f = f;
    }
    public void next(double fx, double dx) {
        x += dx;
        Y += fx * dx;
    }
    public void next(double dx) {
        Y += f.eval(x) * dx;
        x += dx;
    }
    public void integrate(DoubleFunction f, double dx, double x2) {
        this.f = f;
        while (x < x2) {
            next(dx);
        }
    }
}
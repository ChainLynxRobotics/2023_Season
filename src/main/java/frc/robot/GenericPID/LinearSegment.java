package frc.robot.GenericPID;

public class LinearSegment extends PathSegmentBase {
    public double m;
    public double b;
    public double y(double x) {
        return m * x + b;
    }

    public double derivative(double x) {
        return m;
    }

    public LinearSegment(double x1, double y1, double x2, double y2) {
        this.m = (y2 - y1) / (x2 - x1);
        this.b = y1 - m * x1;
    }
}
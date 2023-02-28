package frc.robot.GenericPID;

import java.util.ArrayList;

import frc.robot.GenericPID.Implementations.PathSegmentBase;

public class PolynomialSegment extends PathSegmentBase {
    ArrayList<Double> coeffs; //the index represents the x degree, so [1,2,3] = 3x^2 + 2x + 1
    public double y(double x) {
        double ret = 0;
        for (int i = 0; i < coeffs.size(); i++) {
            ret += coeffs.get(i) * Math.pow(x, i);
        }
        return ret;
    }
    public double derivative(double x) {
        double ret = 0;
        for (int i = 1; i < coeffs.size(); i++) {
            if (i == 1) {
                ret += coeffs.get(i);
            } else {
                ret += coeffs.get(i) * i * Math.pow(x, i - 1);
            }
        }
        return ret;
    }
    public void setCoeff(int power, double value) {
        coeffs.set(power, value);
    }
    public void setDegree(int degree) {
        while (coeffs.size() > degree + 1) { //downsize
            coeffs.remove(coeffs.size() - 1);
        }
        while (coeffs.size() < degree + 1) { //upsize
            coeffs.add(0.0);
        }
    }
    public void generateQuadraticConnector(double x2a, double y2a, double x1b, double y1b, double dydx2a, double dydx1b) {
        //generate a quadratic that connects the two points, with the given derivatives at each point.
        setDegree(2);
        double a = (dydx1b - dydx2a) / (2 * (x1b - x2a));
        double b = dydx2a - 2 * a * x2a;
        double c = y2a - a * x2a * x2a - b * x2a;
        setCoeff(2, a);
    }
    final class Pair {
        public double a;
        public double b;
        public Pair(double a, double b) {
            this.a = a;
            this.b = b;
        }
    }
    public boolean canRedirect(double accel, double x1, double x2, double v1, double v2) {
        //TODO
        return false;
    }
    public void quadraticConnectorSlitDescent() {//maybe : skew it to be more efficient
        return; //TODO
    }

}
package frc.MotionProfiles;

public class PositionRamping {
    private double startPos;
    private double slope;

    public PositionRamping(double rampTime, double setPosition, double startPos) {
        this.startPos = startPos;

        this.slope = setPosition/rampTime;
    }

    public double evaluate(double t)
    {
        return this.slope * t + startPos;
    }
}

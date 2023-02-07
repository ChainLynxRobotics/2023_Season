public class ArtificalMotor {
    //An artificial motor class used for simulation
    private final double mass = 2; //SI, kg
    private final double kdrag = 0.1; //SI, unitless
    private final double kkdrag = 0.1; //SI, units?
    private final double force = 1; //SI, N
    //the artifical motor is simulated with tangential physics, not rotational
    //So anytime the motor has a force applied, kdrag * v is subtracted, and 
    //Acceleration is calculated by mass

    private ApproximateIntegral velocity = new ApproximateIntegral(0, 0);
    private ApproximateIntegral position = new ApproximateIntegral(0, 0);
    private double t;
    public go(double dt) {
        netdrag = kdrag * velocity + kkdrag * velocity * abs(velocity); //always matches the sign of velocity, forces are added
        if (abs(netdrag) > abs(force)) {
            netdrag = force;
        }
        velocity.next( (force - kdrag * velocity - kdrag * velocity * abs(velocity)) / mass, dt);
        position.next(velocity, dt);
        t += dt;
    }
    public slow(double dt) {
        velocity.next(-(force - kdrag * velocity) / mass, dt);
        position.next(velocity, dt);
        t += dt;
    }
    public active(double dt) {
        go(0, dt);
        t += dt;
    }

    public run(double v, double dt, double tlast) {
        while(t < tlast) {
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
    }

    public getVelocity() {
        return velocity;
    }
    public t() {
        return t;
    }
    public ArtificalMotor(double mass, double kdrag, double force) {
        this.mass = mass;
        this.kdrag = kdrag;
        this.force = force;
    }
}
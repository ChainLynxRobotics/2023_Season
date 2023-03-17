package frc.robot.GenericPID.Tuning;

import java.util.ArrayList;

import frc.robot.GenericPID.MotorControlProfile;
import frc.robot.GenericPID.PIDConfig;
import frc.robot.GenericPID.Extensible.DoubleFunction;
import frc.robot.GenericPID.Testing.Graph;
import frc.robot.GenericPID.Testing.Graph.GraphConfig;

public class Tuner {
    private static final double Graph = 0;
    private ArrayList<PIDConfig> trials;

    public Tuner(PIDConfig initial, DoubleFunction runMotorControl, MotorControlProfile control, ArrayList<Double> targets) {
        trials.add(initial);
        GraphConfig gc = new GraphConfig();
        Graph G = new Graph(gc);
    }
    public void beginCLI() {

    }

}

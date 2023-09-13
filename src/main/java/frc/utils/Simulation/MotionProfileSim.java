package frc.utils.Simulation;

import java.awt.Color;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.Lib.MotionProfiles.SCurveProfile;
import frc.Lib.MotionProfiles.TrapezoidalProfile;
import frc.utils.Simulation.Graph.GraphConfig;

public class MotionProfileSim {
    private static GraphConfig config = new Graph.GraphConfig();
    private static Graph testGraph = new Graph(config);
    private static Graph profileGraph = new Graph(config);
    private static double dt = 0.02; //timestep

    private static TrapezoidProfile testProfile = new TrapezoidProfile(new Constraints(5, 10), new TrapezoidProfile.State(10, 0), new TrapezoidProfile.State(0, 0));
    private static TrapezoidalProfile tProfile = new TrapezoidalProfile(5, 10, new TrapezoidalProfile.Config(10, 0));
    private static SCurveProfile sProfile = new SCurveProfile(5, 10, new SCurveProfile.SConfig(10, 0, 0));

    public static void main(String[] args) {
        trapezoidalProfileState();
    }

    public static void trapezoidalProfileState() {
        profileGraph.addPlot(Color.GREEN); //velocity
        profileGraph.addPlot(Color.MAGENTA); //position
        profileGraph.addPlot(Color.BLUE);
        profileGraph.addPlot(Color.GRAY);
        for (double t = 0; t < testProfile.totalTime(); t += dt) {
            SCurveProfile.Config config = sProfile.calculate(t);
            profileGraph.addPoint(t, config.velocity, 0);
            profileGraph.addPoint(t, config.position, 1);
            profileGraph.addPoint(t, testProfile.calculate(t).velocity, 2);
            profileGraph.addPoint(t, testProfile.calculate(t).position, 3);
        }
        profileGraph.init(500, 500, "Motion Profile -- Velocity, Position");
    }

    public static void test() {
        testGraph.addPlot(Color.BLACK);
        testGraph.addPlot(Color.BLUE);
        for (double t = 0; t < 10; t += dt) {
            //plot multiple functions
            testGraph.addPoint(t, Math.cos(t), 0);
            testGraph.addPoint(t, Math.sin(t), 1);
        }
        //initalize graph with dimensions and name
        testGraph.init(500, 500, "test plot");
    }
    
}

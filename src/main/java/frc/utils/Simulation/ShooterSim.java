package frc.utils.Simulation;

import java.awt.Color;

import frc.utils.Simulation.Graph.GraphConfig;


public class ShooterSim {
    private static GraphConfig config = new Graph.GraphConfig();
    private static Graph graph = new Graph(config);

    public static void main(String[] args) {
        test();
    }

    public static void shooterVelocityAndAngleSim() {
        //put your code here!
        //hint: use wpilib motor and encoder simulation library in conjunction with the shooter subsystem to
        //graph how flywheel velocity and hood angle vary over time
    }


    public static void test() {
        double dt = 0.02; //timestep
        graph.addPlot(Color.BLACK);
        graph.addPlot(Color.BLUE);
        for (double t = 0; t < 10; t += dt) {
            //plot multiple functions
            graph.addPoint(t, Math.cos(t), 0);
            graph.addPoint(t, Math.sin(t), 1);
        }
        //initalize graph with dimensions and name
        graph.init(500, 500, "test plot");
    }
}

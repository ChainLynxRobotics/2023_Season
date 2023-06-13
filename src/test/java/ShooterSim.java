package src.test.java;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;

import java.awt.Color;
import java.beans.Transient;

import com.revrobotics.CANSparkMax;

import frc.utils.Simulation.Graph;
import frc.utils.Simulation.Graph.GraphConfig;
import frc.robot.Subsystems.ShooterSubsystem;


public class ShooterSim {
    private static GraphConfig config = new Graph.GraphConfig();
    private static Graph graph = new Graph(config);
    private static double dt = 0.02; //timestep

    private static ShooterSubsystem shooter;


    @BeforeEach
    public void setup() {
        HAL.initialize(100, 0);
        shooter = new ShooterSubsystem(true);
    }


    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach 
    //destroy all subsystem objects here
    public void cleanup() throws Exception {

    }

    @Test
    public static void shooterVelocityAndAngleSim() {
        //put your code here!
        //hint: use wpilib simulation library in conjunction with the shooter subsystem to graph how flywheel velocity and hood angle vary over time
        double setpointVel = 5;

        graph.addPlot(Color.GRAY);

        CANSparkMax motor = shooter.getFlywheelMotor();
        for (double t = 0; t < 5; t += dt) {
            shooter.updateRPM(setpointVel);
            graph.addPoint(t, motor.getAppliedOutput(), 0);
        }

        graph.init(500, 500, "Flywheel Motor Output");
    }


    @Test
    public void testAssertion() {
        assertTrue(true);
    }

    public static void test() {
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


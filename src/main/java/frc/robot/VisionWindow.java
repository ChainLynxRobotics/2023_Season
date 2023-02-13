package frc.robot;

import java.awt.Graphics;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class VisionWindow extends JPanel implements ActionListener {
    public static JFrame frame;
    public static VisionWindow window;
    public static Robot robot;
    public static final int WINDOW_WIDTH = 500;
    public static final int WINDOW_HEIGHT = 500;
    
    public static void startWindow(Robot robot) {
        VisionWindow.robot = robot;
        frame = new JFrame("vision output");
        window = new VisionWindow();
        frame.add(window);
        frame.setSize(WINDOW_WIDTH, WINDOW_HEIGHT);
        frame.setVisible(true);
        Timer t = new Timer(1, window);
        t.start();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        double[] pose = robot.getPose();
        if (pose != null) {
            g.drawString(pose[0]+ " " + pose[1] + " " + pose[2], 100, 100);
            g.fillRect(WINDOW_WIDTH/2+(int)(pose[1]*100), WINDOW_HEIGHT/2+(int)(pose[2]*100), 20, 20);
        }
    }

    @Override
    public void actionPerformed(ActionEvent arg0) {
        repaint();
    }
}
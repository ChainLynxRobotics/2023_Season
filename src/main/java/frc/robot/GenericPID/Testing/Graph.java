package frc.robot.GenericPID.Testing;
// package frc.robot.GenericPID.Approximations.Testing;
//import frc.robot.GenericPID.Testing.*;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import javax.swing.SwingUtilities;
import javax.swing.JFrame;
import javax.swing.JPanel;
import java.lang.Math;

import java.util.ArrayList; 


class Pointp {
    public double x;
    public double y;
}


public class Graph extends JPanel {
    //a 2d cartesian graph object that also drives graphics to represent it. use init and disp to render.
    private GraphConfig config;
    private ArrayList<Pointp> graph;
    
    private JFrame frame;
    private Color color;
    private int pxw;
    private int pxh;

    public static class GraphConfig {
        public double x1; //currently UB if x1 > x2, same with y
        public double x2;
        public double y1;
        public double y2;
        public double xScale;
        public double yScale;
        public double minpps = 5; //minimum pixels per scale
        
        public GraphConfig(double x1, double x2, double y1, double y2, double xScale, double yScale, double minpps) {
            this.x1 = x1;
            this.x2 = x2;
            this.y1 = y1;
            this.y2 = y2;
            this.xScale = xScale;
            this.yScale = yScale;
        }
        public GraphConfig() {
            this(-10., 10., -10., 10., 1., 1., 5.);
        }
    }

    public Graph(GraphConfig c) { //scale only represents the lines in the back showing the values
        this.config = c;
        this.graph = new ArrayList<Pointp>();
        this.frame = null;
        this.pxw = 500;
        this.pxh = 500;
    }

    public void addPoint(double x, double y, double eb, double sb) {
        Pointp p = new Pointp();
        p.x = x;
        p.y = y;
        graph.add(p);
        if (p.x > config.x2) {
            extendBy(eb);
        }
        if (p.y > config.y2) {
            config.y2 = p.y;
        }
        if (p.y < config.y1) {
            config.y1 = p.y;
        }
        if (ppsx() < config.minpps) {
            xscaleBy(sb);
        }
        if (ppsy() < config.minpps) {
            yscaleBy(sb);
        }
    }

    public void addPoint(double x, double y) {
        addPoint(x, y, 5., 2.);
    }

    public void extendBy(double x) {
        config.x2 += x;
    }
    public void xscaleBy(double x) { //scale only represents the lines in the back showing the values
        config.xScale *= x;
    }
    public void yscaleBy(double y) {
        config.yScale *= y;
    }
    public double ppsx() {
        return pxw * config.xScale / (config.x2 - config.x1); //pixels per window * window per x * x per scale = pixels per scale
    }
    public double ppsy() {
        return pxh * config.yScale / (config.y2 - config.y1); //pixels per window * window per y * y per scale = pixels per scale
    }
    public double map(double a1, double a2, double b1, double b2, double n) {
        return (n - a1) / (a2 - a1) * (b2 - b1) + b1; //don't use b1 and b2 swapped, doesn't work correctly, because adds bottom of range (maybe works? idek)
    }
    public int mapx(double n) {
        return (int)map(config.x1, config.x2, 0, pxw, n);
    }
    public int mapy(double n) {
        return (int)map(config.y1, config.y2, 0, pxh, n); //todo fix this it's upside down but if i upside down it fills :((
    }


    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        drawGraph(g);
    }
    public void drawGraph(Graphics g) {
        double x1 = config.x1;
        double x2 = config.x2;
        double y1 = config.y1;
        double y2 = config.y2;
        double xratio = x1 / (x1 - x2);
        double yratio = y1 / (y1 - y2);
        var center = new Point((int)(pxw * xratio), (int)(pxh * yratio));
        
        g.setColor(Color.GRAY.brighter());
        if (x2 > 0) {
            for (int i = 0; i < x2; i += config.xScale) {
                int x = mapx(i);
                g.drawLine(x,0,x,pxh);
            }
        }
        if (x1 < 0) {
            for (int i = 0; i > x1; i -= config.xScale) {
                int x = mapx(i);
                g.drawLine(x,0,x,pxh);
            }
        }
        if (y2 > 0) {
            for (int i = 0; i < y2; i += config.yScale) {
                int y = mapy(i);
                g.drawLine(0,y,pxw,y);
            }
        }
        if (y1 < 0) {
            for (int i = 0; i > y1; i -= config.yScale) {
                int y = mapy(i);
                g.drawLine(0,y,pxw,y);
            }
        }

        g.setColor(Color.BLACK);
        g.drawLine(center.x, 0, center.x, pxh);
        g.drawLine(0, center.y, pxw, center.y);

        g.setColor(color);
        for (int i = 1; i < graph.size(); i++) {
            g.drawLine(mapx(graph.get(i-1).x), mapy(graph.get(i-1).y), mapx(graph.get(i).x), mapx(graph.get(i).y));
        }
    }

    public void init(int pxw, int pxh, String name, Color c) {
        this.pxw = pxw;
        this.pxh = pxh;
        this.frame = new JFrame(name);
        this.color = c;
        this.frame.setSize(pxw, pxh);
        this.frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.frame.getContentPane().add(this, BorderLayout.CENTER);
        this.frame.setVisible(true);
    }

    /**
     * A little driver in case you want a stand-alone application.
     */
    public static void main(String[] args) {
        GraphConfig conf = new Graph.GraphConfig();
        Graph example = new Graph(conf);
        double dt = 0.01;
        for(double t = -3*3.14159; t < 3*3.14159; t+=dt) {
            example.addPoint(t, Math.sin(t));
        }
        example.init(500, 500, "Graph", Color.RED); //TODO: Address that this doesn't include the head of the graph
    }
}

















































































































































// public class Graph {
//     //a 2d cartesian graph object that also drives graphics to represent it. use init and disp to render.
//     public class GraphConfig {
//         public int pxw;
//         public int pxh;
//         public double x1;
//         public double x2;
//         public double y1;
//         public double y2;
//         public double xScale;
//         public double yScale;
//         public double minpps = 5; //minimum pixels per scale

//         public GraphConfig(int pxw, int pxh, double x1, double x2, double y1, double y2, double xScale, double yScale, double minpps) {
//             this.pxw = pxw;
//             this.pxh = pxh;
//             this.x1 = x1;
//             this.x2 = x2;
//             this.y1 = y1;
//             this.y2 = y2;
//             this.xScale = xScale;
//             this.yScale = yScale;
//         }

//         public GraphConfig() {
//             this(500., 500., -10., 10., -10., 10., 1., 1., 5.);
//         }
//     }
    
//     private GraphConfig config;
//     private ArrayList<Pointp> graph;

//     public Graph(GraphConfig c) { //scale only represents the lines in the back showing the values
//         this.config = c;
//         this.graph = new ArrayList<Pointp>();
//     }

//     public void addPoint(double x, double y, double eb, double sb) {
//         Pointp p = new Pointp();
//         p.x = x;
//         p.y = y;
//         graph.add(p);
//         if (p.x > config.x2) {
//             extendBy(eb);
//         }
//         if (p.y > config.y2) {
//             config.y2 = p.y;
//         }
//         if (p.y < config.y1) {
//             config.y1 = p.y;
//         }
//         if (ppsx() < config.minpps) {
//             xscaleBy(sb);
//         }
//         if (ppsy() < config.minpps) {
//             yscaleBy(sb);
//         }
//     }

//     public void addPoint(double x, double y) {
//         addPoint(x, y, 5., 2.);
//     }

//     public void extendBy(double x) {
//         x2 += x;
//     }
//     public void xscaleBy(double x) { //scale only represents the lines in the back showing the values
//         xScale *= x;
//     }
//     public void yscaleBy(double y) {
//         yScale *= y;
//     }

//     public double ppsx() {
//         return pxw * xscale/ (x2 - x1); //pixels per window * window per x * x per scale = pixels per scale
//     }

//     public double ppsy() {
//         return pxh * yscale/ (y2 - y1); //pixels per window * window per y * y per scale = pixels per scale
//     }
// }
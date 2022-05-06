package com.titanrobotics2022.demos;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import javax.swing.JFrame;
import javax.swing.JPanel;

import com.titanrobotics2022.mapping.Path;
import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;
import com.titanrobotics2022.motion.generation.rmpflow.rmps.CollisionAvoidance;
import com.titanrobotics2022.motion.generation.rmpflow.rmps.PathFollowing;
import com.titanrobotics2022.mapping.LinearSegment;
import com.titanrobotics2022.mapping.Point;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

public class RMPDemo {

    public RMPDemo() {

        double step = 0.02; // Default: 0.02
        double speed = 0.8; // [0.0, 1.0]

        RMPRoot root = new RMPRoot("root");
        SimpleMatrix x = new SimpleMatrix(1, 2, false, new double[] { 50, 50 });
        SimpleMatrix x_dot = new SimpleMatrix(1, 2, false, new double[] { 0, 0 });
        SimpleMatrix x_ddot = new SimpleMatrix(1, 2, false, new double[] { 0, 0 });
        double v = 8, P = 5, I = 0, A = 1, B = 0.5, K = 1, h = 0.5, maxAcc = 2;

        SimpleMatrix goal = new SimpleMatrix(1, 2, false, new double[] { 250, 350 });
        Path path = new LinearSegment(new Point(x.get(0), x.get(1)), new Point(goal.get(0), goal.get(1)));
        PathFollowing follower = new PathFollowing("Path Following Demo", root, path, v, P, I, A, B, K, h, maxAcc);

        ArrayList<CollisionAvoidance> obstacles = new ArrayList<>();
        obstacles.add(new CollisionAvoidance("Obstacle 1", root,
                new SimpleMatrix(1, 2, false, new double[] { 100, 100 }), 10,
                1, 1, 1));
        obstacles.add(new CollisionAvoidance("Obstacle 2", root,
                new SimpleMatrix(1, 2, false, new double[] { 150, 200 }), 15, 1, 1, 1));
        obstacles.add(new CollisionAvoidance("Obstacle 3", root,
                new SimpleMatrix(1, 2, false, new double[] { 200, 300 }), 20,
                1, 1, 1));

        ArrayList<Double> simulationData = new ArrayList<Double>();
        JFrame frame = new JFrame("Path Following Demo");
        frame.setSize(500, 500);
        frame.setVisible(true);

        JPanel panel = new JPanel() {
            @Override
            public void paintComponent(Graphics g) {
                super.paintComponent(g);
                Graphics2D g2 = (Graphics2D) g;
                g2.setColor(Color.red); // obs
                for (CollisionAvoidance obs : obstacles)
                    g2.fillOval((int) (obs.getCenter().get(0) / 1),
                            (int) (obs.getCenter().get(1) / 1), (int) obs.getRadius(), (int) obs.getRadius());
                g2.fillOval((int) (simulationData.get(0) / 1),
                        (int) (simulationData.get(1) / 1), 10, 10);
                g2.setColor(Color.green); // start
                g2.fillOval((int) (simulationData.get(0) / 1),
                        (int) (simulationData.get(1) / 1), 10, 10);
                for (int i = 6; i < simulationData.size(); i += 6) { // path
                    g2.setColor(Color.black);
                    g2.fillOval((int) (simulationData.get(i) / 1),
                            (int) (simulationData.get(i + 1) / 1), 3, 3);
                }
                g2.setColor(Color.blue); // goal
                g2.fillOval((int) (goal.get(0)),
                        (int) (goal.get(1)), 10, 10);
                // state metrics
                g2.drawString("x0: " + simulationData.get(simulationData.size() - 6), 300, 20);
                g2.drawString("x1: " + simulationData.get(simulationData.size() - 5), 300, 40);
                g2.drawString("x_dot0: " + simulationData.get(simulationData.size() - 4), 300, 60);
                g2.drawString("x_dot1: " + simulationData.get(simulationData.size() - 3), 300, 80);
                g2.drawString("x_ddot0: " + simulationData.get(simulationData.size() - 2), 300, 100);
                g2.drawString("x_ddot1: " + simulationData.get(simulationData.size() - 1), 300, 120);
                g2.drawString("t: " + simulationData.size() / 6 * step, 300, 140);
                g2.drawRect((int) (simulationData.get(simulationData.size() - 6) / 1),
                        (int) (simulationData.get(simulationData.size() - 5) / 1), 10, 10);
            }
        };
        frame.add(panel);

        double tolerance = 0.1;
        int MAX_ITER = 100000;
        for (int i = 0; Math.abs(x.minus(goal).normF()) > tolerance && i < MAX_ITER; i++) {
            x_ddot = root.solve(x, x_dot);
            // System.out.printf("x: (%f, %f)\n", x.get(0), x.get(1));
            // System.out.printf("x_dot: (%f, %f)\n", x_dot.get(0), x_dot.get(1));
            // System.out.printf("x_ddot: (%f, %f)\n", x_ddot.get(0), x_ddot.get(1));
            // System.out.printf("t: %f\n", step * i);
            double[] newState = solveIntegration(step, x_ddot, x_dot, x);
            x_dot.set(1, newState[3]);
            x_dot.set(0, newState[2]);
            x.set(1, newState[1]);
            x.set(0, newState[0]);
            simulationData.add(newState[0]);
            simulationData.add(newState[1]);
            simulationData.add(newState[2]);
            simulationData.add(newState[3]);
            simulationData.add(x_ddot.get(0));
            simulationData.add(x_ddot.get(1));
            panel.repaint();
            try {
                Thread.sleep((long) (100 - (speed / 10 + 0.9) * 100));
            } catch (InterruptedException e) {
            }
        }

    }

    public static double[] solveIntegration(double deltaT, SimpleMatrix x_ddot, SimpleMatrix x_dot, SimpleMatrix x) {
        double[] state = new double[4];
        state[3] = x_dot.get(1) + x_ddot.get(1) * deltaT;
        state[2] = x_dot.get(0) + x_ddot.get(0) * deltaT;
        state[1] = x.get(1) + .5 * (x_dot.get(1) + state[3]) * deltaT;
        state[0] = x.get(0) + .5 * (x_dot.get(0) + state[2]) * deltaT;
        return state;
    }

    public static void main(String[] args) {
        new RMPDemo();
    }

}

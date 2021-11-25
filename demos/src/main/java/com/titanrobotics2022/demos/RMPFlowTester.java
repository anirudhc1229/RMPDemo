package com.titanrobotics2022.demos;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;

import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;
import com.titanrobotics2022.motion.generation.rmpflow.rmps.CollisionAvoidance;
import com.titanrobotics2022.motion.generation.rmpflow.rmps.GoalAttractor;

import org.ejml.simple.SimpleMatrix;

public class RMPFlowTester{

	public RMPFlowTester() {
		//RMP_example
		double alpha = 1e-5, eta = 2, epsilon = .2;
		//double alpha = 1e-3, eta = 4, epsilon = 1;
		RMPRoot r = new RMPRoot("root");
		ArrayList<CollisionAvoidance> collisionAvoiders = new ArrayList<CollisionAvoidance>();
		ArrayList<GoalAttractor> goalAttractors = new ArrayList<GoalAttractor>();
		
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test", r, new SimpleMatrix(1, 2,  false, new double[] {0, 0}), 1, .2, 1e-5, 0));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test2", r, new SimpleMatrix(1, 2, false, new double[] {1, 1}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test2", r, new SimpleMatrix(1, 2, false, new double[] {2, 2.5}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test2", r, new SimpleMatrix(1, 2, false, new double[] {4, 4}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test2", r, new SimpleMatrix(1, 2, false, new double[] {-.5, -5}), 1, epsilon, alpha, eta));
		
		/* Bad Behavior Demo
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test2", r, new SimpleMatrix(1, 2, false, new double[] {-4, 0}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test3", r, new SimpleMatrix(1, 2, false, new double[] {-3, 1}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test4", r, new SimpleMatrix(1, 2, false, new double[] {-2, 2}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test5", r, new SimpleMatrix(1, 2, false, new double[] {-1, 2}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test6", r, new SimpleMatrix(1, 2, false, new double[] {0, 2}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test6", r, new SimpleMatrix(1, 2, false, new double[] {1, 2}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test6", r, new SimpleMatrix(1, 2, false, new double[] {2, 2}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test6", r, new SimpleMatrix(1, 2, false, new double[] {3, 1}), 1, epsilon, alpha, eta));
		collisionAvoiders.add(new CollisionAvoidance("Collision Avoidance Test6", r, new SimpleMatrix(1, 2, false, new double[] {4, 0}), 1, epsilon, alpha, eta));
		*/
		goalAttractors.add(new GoalAttractor("Goal Attractor Test", r, new SimpleMatrix(1, 2, false, new double[] {0, 10}), 10, 1, 10, 1, 2, 2, .005));
		
		SimpleMatrix x = new SimpleMatrix(1, 2, false, new double[] {0.5, -10});
		SimpleMatrix x_dot = new SimpleMatrix(1, 2, false, new double[] {0, 0});
		SimpleMatrix x_ddot;
		
		//Simulation
		ArrayList<Double> simulationData = new ArrayList<Double>();
		for(int i = 0; i < 4500/5; i++)
		{
			x_ddot = r.solve(x, x_dot);
			double[] newState = solveIntegration(.01*5, x_ddot, x_dot, x);
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
		}
		
		for(int i = 0; i < simulationData.size(); i+=6)
		{
			System.out.println(simulationData.get(i) + "\t" + simulationData.get(i + 1)
					//+ simulationData.get(i + 2) + "\t" + simulationData.get(i + 3)
					//+ simulationData.get(i + 4) + "\t" + simulationData.get(i + 5)
					);
		}
		
		//Searching Data for bounds
		double xL = 0, xH = 0, yL = 0, yH = 0, vL = 0, vH = 0, aL = 0, aH = 0;
		for(int i = 0; i < simulationData.size(); i+=6)
		{
			if(simulationData.get(i) < xL)
			{
				xL = simulationData.get(i);
			}
			if(simulationData.get(i) > xH)
			{
				xH = simulationData.get(i);
			}
			if(simulationData.get(i + 1) < yL)
			{
				yL = simulationData.get(i + 1);
			}
			if(simulationData.get(i + 1) > yH)
			{
				yH = simulationData.get(i + 1);
			}
			if(getMagnitude(simulationData.get(i + 2), simulationData.get(i + 3)) < vL)
			{
				vL = getMagnitude(simulationData.get(i + 2), simulationData.get(i + 3));
			}
			if(getMagnitude(simulationData.get(i + 2), simulationData.get(i + 3)) > vH)
			{
				vH = getMagnitude(simulationData.get(i + 2), simulationData.get(i + 3));
			}
			if(getMagnitude(simulationData.get(i + 4), simulationData.get(i + 5)) < aL)
			{
				aL = getMagnitude(simulationData.get(i + 4), simulationData.get(i + 5));
			}
			if(getMagnitude(simulationData.get(i + 4), simulationData.get(i + 5)) > aH)
			{
				aH = getMagnitude(simulationData.get(i + 4), simulationData.get(i + 5));
			}
			
		}
		
		//Displaying data
		JFrame f = new JFrame("RMPFlow Example");
		f.setVisible(true);
		f.setSize(500, 500);
		//f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setResizable(false);
		
		final double xLf = xL, xHf = xH, yLf = yL, yHf = yH, vLf = vL, vHf = vH, aLf = aL, aHf = aH;
		JPanel panel = new JPanel() {
            @Override
            public void paintComponent(Graphics g) {
                super.paintComponent(g);
                Graphics2D g2 = (Graphics2D) g;
                //double ratio = Math.abs((yLf + yHf)/(xLf + xHf));
                g2.draw(new Rectangle2D.Double(20, 20, 400, 400));
                int circleDiameter = 3;
                for(int i = 0; i < simulationData.size(); i+=6)
                {
                	g2.setColor(Color.black);
                	g2.setColor(new Color(0, 0, (int) (getMagnitude(simulationData.get(i + 2), simulationData.get(i + 3)) / vHf * 255)));
                	g2.fillOval((int) (20 + 400/2 + simulationData.get(i)/10 * 200), (int) (20 + 400/2 - simulationData.get(i + 1)/10 * 200), circleDiameter, circleDiameter);
                }
                for(int i = 0; i < collisionAvoiders.size(); i++)
                {
                	g2.setColor(Color.red);
                	g2.fillOval((int) (20 + 400/2 + collisionAvoiders.get(i).getCenter().get(0)/10 * 200 - (collisionAvoiders.get(i).getRadius()/10 * 100))
                			, (int) (20 + 400/2 - collisionAvoiders.get(i).getCenter().get(1)/10 * 200)
                			, (int) (collisionAvoiders.get(i).getRadius()/10 * 200)
                			, (int) (collisionAvoiders.get(i).getRadius()/10 * 200));
                }
            }
        };
        f.add(panel);
		
	}
	
	public static double getMagnitude(double x, double y)
	{
		return Math.sqrt(x * x + y * y);
	}

	//Dynamics of system
	public static double[] solveIntegration(double deltaT, SimpleMatrix x_ddot, SimpleMatrix x_dot, SimpleMatrix x)
	{
		double[] state = new double[4];
		state[3] = x_dot.get(1) + x_ddot.get(1) * deltaT;//y value
		state[2] = x_dot.get(0) + x_ddot.get(0) * deltaT;//x value
		state[1] = x.get(1) + .5 * (x_dot.get(1) + state[3]) * deltaT;//Trapezoid integration
		state[0] = x.get(0) + .5 * (x_dot.get(0) + state[2]) * deltaT;
		return state;
	}
}

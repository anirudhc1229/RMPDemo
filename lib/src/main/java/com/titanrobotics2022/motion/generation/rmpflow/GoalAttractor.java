package frc.robot.motion.generation.rmpflow;

import org.ejml.simple.SimpleMatrix;

/**
 * Implementation of a 2D Goal Attractor using riemannian motion policies.
 * 
 * From: <a href="https://arxiv.org/abs/1902.05177">Multi-Objective Policy Generation for Multi-Robot Systems Using Riemannian Motion Policies</a>
 */
public class GoalAttractor extends RMPLeaf {
	private double w_u, w_l, sigma, alpha, eta, gain , tolerance;
	private SimpleMatrix goal, jeye;
	private int goalSize;
	
	public GoalAttractor(String name, RMPNode parent, SimpleMatrix goal, double w_u, double w_l, double sigma
			, double alpha, double eta, double gain, double tolerance)
	{
		super(name, parent);
		this.w_u = w_u;
		this.w_l = w_l;
		this.sigma = sigma;
		this.alpha = alpha;
		this.eta = eta;
		this.gain = gain;
		this.tolerance = tolerance;
		
		if(goal.numRows() == 1)
			this.goal = goal.transpose();
		else
			this.goal = goal;
		
		updateGoal(this.goal);
		goalSize = this.goal.numCols() * this.goal.numRows();
		jeye = SimpleMatrix.identity(goalSize);
	}

	public SimpleMatrix psi(SimpleMatrix q)
	{
		return q.minus(goal);
	}

	public SimpleMatrix j(SimpleMatrix q)
	{
		return jeye;
	}

	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot)
	{
		return new SimpleMatrix(goalSize, goalSize);
	}
	
	public SimpleMatrix solveF(SimpleMatrix x, SimpleMatrix x_dot)
	{
		double x_norm = x.normF();
		
		double beta = Math.exp(- Math.pow(x_norm, 2) / 2 / Math.pow(sigma, 2));
		double w = (w_u - w_l) * beta + w_l;
		double s = (1 - Math.exp(-2 * alpha * x_norm)) / (1 + Math.exp(-2 * alpha * x_norm));
		
		SimpleMatrix grad_Phi;
		if(x_norm > tolerance)
			grad_Phi = x.scale(s / x_norm * w * gain);
		else
			grad_Phi = new SimpleMatrix(x.numRows() , 1);
		SimpleMatrix bx_dot = x_dot.scale(eta * w);
		SimpleMatrix grad_w = x.scale(-beta * (w_u - w_l) / Math.pow(sigma, 2));
		
		double x_dot_norm = x_dot.normF();
		SimpleMatrix xi = grad_w.scale(Math.pow(x_dot_norm, 2)).minus(x_dot.mult(x_dot.transpose()).mult(grad_w).scale(2)).scale(-.5);
		
		return grad_Phi.scale(-1).minus(bx_dot).minus(xi);
	}
	
	public SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot)
	{
		double x_norm = x.normF();
		
		double beta = Math.exp(- Math.pow(x_norm, 2) / 2 / Math.pow(sigma, 2));
		double w = (w_u - w_l) * beta + w_l;
		
		SimpleMatrix G = SimpleMatrix.identity(this.goal.numCols() * this.goal.numRows()).scale(w);

		return G;
	}
	
	public void updateGoal(SimpleMatrix goal)
	{
		if(goal.numRows() == 1)
			goal = goal.transpose();
		this.goal = goal;
	}
}

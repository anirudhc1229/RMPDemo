package frc.robot.motion.generation.rmpflow;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * A 1-d distance subtask space collision avoidance between two points.
 * Implementation of Section 3.1 Pairwise Collision Avoidance from <a href="https://arxiv.org/abs/1902.05177">Multi-Objective Policy Generation for Multi-Robot Systems Using Riemannian Motion Policies</a>
 * 
 * @implNote There is no orientation component only R2
 */
public class CollisionAvoidance extends RMPLeaf {
	// r(dS): a minimum safety distance between points
	// alpha(α): positive potenial function scalar
	// eta(η): positive damping matrix scalar
	// epsilon(ɛ): small positive scalar for avoidance metric
	private double r, alpha, eta, epsilon;
	private SimpleMatrix center; // location of center of circular obstacle
	
	/**
	 * A 1-d distance subtask space collision avoidance RMP Node between obstacle with radius r.
	 * @param name The name of the motion policy.
	 * @param parent The parent node of current RMP Node.
	 * @param center The location of the center of the circular obstacle.
	 * @param r The radius of the obstacle.
	 * @param epsilon The positive damping matrix scalar
	 * @param alpha The positive potenial function scalar
	 * @param eta A small positive scalar for avoidance metric
	 */
	public CollisionAvoidance(String name, RMPNode parent, SimpleMatrix center, double r, double epsilon, double alpha, double eta)
	{
		super(name, parent);
		this.r = r;
		this.alpha = alpha;
		this.eta = eta;
		this.epsilon = epsilon;
		
		if(center.numRows() == 1)
			this.center = center.transpose();
		else
			this.center = center;
	}

	/**
	 * A 1-d distance subtask space collision avoidance RMP Node between obstacle with radius r.
	 * @param name The name of the motion policy.
	 * @param parent The parent node of current RMP Node.
	 * @param center The location of the center of the circular obstacle.
	 * @param r The radius of the obstacle.
	 * @param epsilon The positive damping matrix scalar
	 * @param alpha The positive potenial function scalar
	 * @param eta A small positive scalar for avoidance metric
	 */
	public CollisionAvoidance(String name, RMPNode parent, Pose2d center, double r, double epsilon, double alpha, double eta)
	{
		this(name, parent, new SimpleMatrix(1, 2,  false, new double[] {center.getX(), center.getY()}), r, epsilon, alpha, eta);
	}

	/**
	 * R^N to R Task Map
	 * 
	 * z = psi(q,center) = ||q - center|| / r - 1
	 * 
	 * @param q An R^N dimensional state
	 * @return 1-d matrix
	 */
	public SimpleMatrix psi(SimpleMatrix q)
	{
		return new SimpleMatrix(1, 1, false, new double[] {q.minus(center).normF() / r - 1}).transpose();
	}

	/**
	 * Jacobian of psi:
	 * ((q - center) / r) * 1 / ||q - center|| 
	 * 
	 * @param q An R^N dimensional state
	 * @return // TODO: Describe what a jacobian respresents in this instance
	 */
	public SimpleMatrix j(SimpleMatrix q)
	{
		double scale = 1/q.minus(center).normF();
		return q.minus(center).transpose().divide(r).scale(scale);
	}

	/**
	 * Derivative of Jacobian of psi:
	 * <p>
	 * T is transpose
	 * <p>
	 * I is a diagonal identity matrix with the same dimensions as center
	 * <p>
	 * a = ((q - center) * (q - center))T
	 * <p>
	 * b = a + I * (1 / ||q - center||)
	 * <p>
	 * j_dot(q, q_dot) = q_dotT * b / r
	 * 
	 * @param q An R^N dimensional state
	 * @param q_dot The derivative of an R^N dimensional state
	 * @return // TODO: Describe what the derivative of a jacobian respresents in this instance
	 */
	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot)
	{
		SimpleMatrix a = q.minus(center).mult(q.minus(center).transpose())
						  	  .scale(-1 / Math.pow(q.minus(center).normF(), 3));
		SimpleMatrix b = a.plus(SimpleMatrix.identity(center.getNumElements()).scale(1 / q.minus(center).normF()));
		return q_dot.transpose().mult(b).divide(r);
	}
	
	/**
	 * Implementation of a barrier-type potential from Section 3.1 Pairwise Collision Avoidance from <a href="https://arxiv.org/abs/1902.05177">Multi-Objective Policy Generation for Multi-Robot Systems Using Riemannian Motion Policies</a>
	 * <p>
	 * G(x, x_dot) = w(x) * u(x_dot)
	 * <p>
	 * w(x) = 1 / x ^ 4
	 * <p>
	 * u(x_dot) = ɛ + min(0, x_dot) * x_dot
	 * <p>
	 * This means that the collision avoidance RMP dominates when robots
	 * are close to each other or moving fast towards each other.
	 * <p>
	 * Φ(x) = .5 * α * w(x)^2
	 * <p>
	 * The GDS defined as a potential function phi.
	 * <p>
	 * B(x, x_dot) = η * G(x, x_dot)
	 * <p>
	 * This is a dampening matrix.
	 * <p>
	 * 
	 * @param x The subtask space state
	 * @param x_dot The subtask space derivative
	 * @return The acceleration motion policy denoted F
	 */
	public SimpleMatrix solveF(SimpleMatrix x, SimpleMatrix x_dot)
	{
		double w;
		double grad_w;
		if(x.get(0, 0) < 0)
		{
			w = 1e10;
			grad_w = 0;
		}
		else
		{
			w = 1 / Math.pow(x.get(0, 0), 4);
			grad_w = -4 / Math.pow(x.get(0, 0), 5);
		}
		
		double u = epsilon + Math.min(0, x_dot.get(0, 0)) * x_dot.get(0, 0);
		double g = w * u;
		
		double grad_Phi = alpha * w * grad_w;
		double xi = .5 * Math.pow(x_dot.get(0, 0), 2) * u * grad_w;
		
		double bx_dot = eta * g * x_dot.get(0, 0);
		
		double f_double = -grad_Phi - xi - bx_dot;
		return new SimpleMatrix(1, 1, false, new double[] {Math.min(Math.max(-1e10, f_double), 1e10)});

	}
	
	/**
	 * Implementation from  <a href="https://github.com/gtrll/multi-robot-rmpflow/blob/master/rmp_leaf.py">Multi-Objective Policy Generation for Multi-Robot Systems Using Riemannian Motion Policies</a>
	 * <p>
	 * G(x, x_dot) = w(x) * u(x_dot)
	 * <p>
	 * w(x) = 1 / x ^ 4
	 * <p>
	 * u(x_dot) = ɛ + min(0, x_dot) * x_dot
	 * <p>
	 * This means that the collision avoidance RMP dominates when robots
	 * are close to each other or moving fast towards each other.
	 * <p>
	 * M = G(x, x_dot) * .5 * x_dot * w * grad_u
	 * <p>
	 * 
	 * @param x The subtask space state
	 * @param x_dot The subtask space derivative
	 * @return The inertia matrix denoted M
	 */
	public SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot)
	{
		double w;
		if(x.get(0, 0) < 0)
		{
			w = 1e10;
		}
		else
		{
			w = 1 / Math.pow(x.get(0, 0), 4);
		}
		
		double u = epsilon + Math.min(0, x_dot.get(0, 0)) * x_dot.get(0, 0);
		double g = w * u;
		
		double grad_u = 2 * Math.min(0, x_dot.get(0, 0));
		
		double m_double = g + .5 * x_dot.get(0, 0) * w * grad_u;
		return new SimpleMatrix(1, 1, false, new double[] {Math.min(Math.max(-1e5, m_double), 1e5)});
	}
	
	/**
	 * Returns radius of the obstacle
	 * @return The radius of obstacle.
	 */
	public double getRadius()
	{
		return r;
	}

	/**
	 * Returns the center of the obstacle
	 * @return The center of the obstacle
	 */
	public SimpleMatrix getCenter()
	{
		return center;
	}
}
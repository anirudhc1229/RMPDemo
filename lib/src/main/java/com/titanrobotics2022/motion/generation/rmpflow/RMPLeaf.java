package frc.robot.motion.generation.rmpflow;

import org.ejml.simple.SimpleMatrix;

/**
 * A leaf node with no children that contains an RMP.
 * Includes implementation as described in the <a href="https://arxiv.org/abs/1811.07049">RMPFlow Section 3.5</a>.
 */
public abstract class RMPLeaf extends RMPNode{
	/**
	 * RMP leaf node containing an RMP.
	 * @param name of leaf node
	 * @param parent of leaf node
	 */
	public RMPLeaf(String name, RMPNode parent)
	{
		super(name, parent);
	}
	
	@Override
	public final void pullback()
	{
		evaluate();
	}

	/**
	 * Solves for the M and F of the RMP.
	 */
	public final void evaluate()
	{	
		setM(solveM(getX(), getXdot()));
		setF(solveF(getX(), getXdot()));
	}

	/**
	 * Solves for F
	 * , where F is the motion policy that describes the dynamical system as a second-order differential equation that uses position and velocity.
	 * @param x The RMPLeaf state in task space
	 * @param x_dot The RMPLeaf differentiated state in task space
	 * @return F, the force motion policy
	 */
	protected abstract SimpleMatrix solveF(SimpleMatrix x, SimpleMatrix x_dot);
	
	/**
	 * Solves for M
	 * , where M is the canonical version of the Riemannian metric A
	 * @param x The RMPLeaf state in task space
	 * @param x_dot The RMPLeaf differentiated state in task space
	 * @return M, the inertia matrix
	 */
	protected abstract SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot);
}

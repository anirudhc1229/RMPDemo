package frc.robot.motion.generation.rmpflow;

import org.ejml.simple.SimpleMatrix;

/**
 * The root node of an RMP tree.
 * Includes implementation as described in the <a href="https://arxiv.org/abs/1811.07049">RMPFlow Section 3.5</a>.
 */
public class RMPRoot extends RMPNode{
	/**
	 * A root node of an RMP tree.
	 * @param name The name of the tree
	 */
	public RMPRoot(String name)
	{
		super(name, null);
	}
	
	/**
	 * Updates the states of the tree
	 * @param x The state
	 * @param x_dot The differentiated state
	 */
	public void setRootState(SimpleMatrix x, SimpleMatrix x_dot)
	{
		if(x.numRows() == 1) //Sets to column matrices
			setX(x.transpose());
		else
			setX(x);
		if(x_dot.numRows() == 1)
			setXdot(x_dot.transpose());
		else
			setXdot(x_dot);
	}
	
	@Override
	public void pushforward()
	{
		for(int i = 0; i < getChildren().size(); i++)
			getChildren().get(i).pushforward();
	}
	
	/**
	 * Maps an RMP from its natural form to its canonical form.
	 * @return A The matrix containing desired acceleration in canonical form.
	 */
	public SimpleMatrix resolve()
	{
		return getA();
	}
	
	/**
	 * Updates the state of the tree and solves for the desired output.
	 * @param x The state
	 * @param x_dot The differentiated state
	 * @return A matrix representing the resolved state of the RMP tree.
	 */
	public SimpleMatrix solve(SimpleMatrix x, SimpleMatrix x_dot)
	{
		setRootState(x, x_dot);
		pushforward();
		pullback();
		return resolve();
	}
}

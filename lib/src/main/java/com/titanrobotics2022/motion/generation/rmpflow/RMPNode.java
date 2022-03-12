package com.titanrobotics2022.motion.generation.rmpflow;

import java.util.ArrayList;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.simple.SimpleMatrix;

/**
 * Represents a single node and its connections in a Riemannian Motion Policies
 * Tree.
 * Includes implementation as described in the
 * <a href="https://arxiv.org/abs/1811.07049">RMPFlow</a> algorithm.
 */
public abstract class RMPNode {
	private String name; // Name of RMP
	private RMPNode parent; // Parent node
	private ArrayList<RMPNode> children = new ArrayList<RMPNode>(); // All child nodes
	private SimpleMatrix x, x_dot, f, m;
	private LinearSolver<DMatrixRMaj, DMatrixRMaj> solver = LinearSolverFactory_DDRM.pseudoInverse(true);

	// See <a href="https://arxiv.org/abs/1811.07049">RMPFlow Section 3.2</a>
	// x: current state
	// x_dot: derivative of current state
	// f: desired force map
	// m: inertia matrix

	/**
	 * A node for an tree that contains mappings and functions for Riemannian Motion
	 * Policies.
	 * 
	 * @param name   of RMP node
	 * @param parent of RMP node
	 */
	public RMPNode(String name, RMPNode parent) {
		this.name = name;
		this.parent = parent;
		if (parent != null) // TODO: Throw null pointer if parent doesn't exist
			parent.linkChild(this);
		this.x = null;
		this.x_dot = null;
		this.f = null;
		this.m = null;
	}

	/**
	 * Differentiable task map that relates the configuration space to the task
	 * space.
	 * Ψ : C &gt; T
	 * 
	 * @param q The configuration space
	 * @return The task space
	 */
	public SimpleMatrix psi(SimpleMatrix q) {
		return q;
	}

	/**
	 * Jacobian of the task map psi
	 * 
	 * @param q The configuration space
	 * @return A transformation via Jacobian of the task map psi
	 */
	public SimpleMatrix j(SimpleMatrix q) {
		return q;
	}

	/**
	 * Second derivative Jacobian of the task map psi
	 * 
	 * @param q     The configuration space
	 * @param q_dot The configuration space
	 * @return A transformation via second-order Jacobian of the task map psi
	 */
	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot) {
		return q_dot;// TODO decide if to implement like goal Attractor J_dot
	}

	/**
	 * Set the current node's parent and updates the old and current link.
	 * 
	 * @param parent A RMP node parent
	 */
	public void linkParent(RMPNode parent) {
		this.parent.removeChild(this);
		parent.children.add(this);
		this.parent = parent;
	}

	/**
	 * 
	 * @param parent
	 */
	public void unlinkParent(RMPNode parent) {
		parent.removeChild(this);
		this.parent = null;
	}

	/**
	 * Adds a node as a child to the implicit node.
	 * 
	 * @param child A RMP node child
	 */
	public void linkChild(RMPNode child) {
		child.linkParent(this);
	}

	/**
	 * 
	 * @param child
	 */
	public void unlinkChild(RMPNode child) {
		child.unlinkParent(this);
	}

	/**
	 * Remove the child node from implicit parent node
	 * 
	 * @implNote Does not update the child's parent connection
	 * @param child The child to remove
	 * @return The child that was removed, returns null if nothing was removed
	 */
	private RMPNode removeChild(RMPNode child) {
		for (int i = 0; i < children.size(); i++) {
			if (children.get(i) == child) // Object reference check
			{
				return children.remove(i);
			}
		}
		return null;
	}

	/**
	 * Returns all child nodes.
	 * 
	 * @return An array list of all children nodes.
	 */
	public ArrayList<RMPNode> getChildren() {
		return children;
	}

	/**
	 * Returns parent node
	 * 
	 * @return RMPNode parent
	 */
	public RMPNode getParent() {
		return parent;
	}

	/**
	 * The operator to forward propagate the state from a parent node to its child
	 * nodes.
	 * I.e. Transform an RMP from the domain of a task map to its co-domain.
	 * Equation 12 in <a href="https://arxiv.org/abs/1801.02854">Riemannian Motion
	 * Policies Section IV</a>.
	 */
	public void pushforward() {
		// In the case that an RMP node does not have a psi or j function implement psi
		// and j to return the input.
		// TODO: Figure out above case
		x = psi(parent.x);// psi(x)
		x_dot = j(parent.x).mult(parent.x_dot);// j(x) * x_dot

		for (int i = 0; i < children.size(); i++)
			children.get(i).pushforward();
	}

	/**
	 * The operator to backward propagate the natural-formed RMPs from the child
	 * nodes to the parent node.
	 * I.e update parent states
	 * Equation 11 in <a href="https://arxiv.org/abs/1801.02854">Riemannian Motion
	 * Policies Section IV</a>.
	 */
	public void pullback() {
		for (int i = 0; i < children.size(); i++)
			children.get(i).pullback();

		f = x.copy();
		f.zero();

		int mSize = Math.max(x.numCols(), x.numRows());
		if (m == null)
			m = new SimpleMatrix(mSize, mSize);
		else
			m.reshape(mSize, mSize);
		m.zero();

		for (int i = 0; i < children.size(); i++) {
			RMPNode child = children.get(i);
			SimpleMatrix J_child = child.j(x);// child's Jacobian of x
			SimpleMatrix J_dot_child = child.j_dot(x, x_dot);

			if (child.f != null && child.name != null) {
				// f + JT * (f - (m * J_dot * x_dot)) Equation 1 in RMPFlow Computational Graph
				f = f.plus(J_child.transpose().mult(child.f.minus(
						child.m.mult(J_dot_child).mult(x_dot))));
				// JT * M * J Equation 1 in RMPFlow Computational Graph
				m = m.plus(J_child.transpose().mult(child.m).mult(J_child));
			}
		}
	}

	/**
	 * Returns the RMP's current task space state
	 * 
	 * @return x The RMP's current task space state
	 */
	public SimpleMatrix getX() {
		return x;
	}

	/**
	 * Sets the RMP's current task space state
	 * 
	 * @param x The RMP's current task space state
	 */
	public void setX(SimpleMatrix x) {
		this.x = x;
	}

	/**
	 * Gets the RMP's current task space derivative state
	 * 
	 * @return RMP's current task space derivative state
	 */
	public SimpleMatrix getXdot() {
		return x_dot;
	}

	/**
	 * Sets the RMP's current task space derivative state
	 * 
	 * @param xdot The RMP's current task space derivative state
	 */
	public void setXdot(SimpleMatrix xdot) {
		this.x_dot = xdot;
	}

	/**
	 * Returns the force motion policy
	 * 
	 * @return f The force motion policy
	 */
	public SimpleMatrix getF() {
		return f;
	}

	/**
	 * Sets the force motion policy
	 * 
	 * @param f The force motion policy
	 */
	public void setF(SimpleMatrix f) {
		this.f = f;
	}

	/**
	 * Returns inertia matrix
	 * 
	 * @return M The inertia matrix
	 */
	public SimpleMatrix getM() {
		return m;
	}

	/**
	 * Sets inertia matrix
	 * 
	 * @param m The inertia matrix
	 */
	public void setM(SimpleMatrix m) {
		this.m = m;
	}

	/**
	 * Computes and returns motion policy as a = M†f.
	 * † denotes Moore-Penrose inverse.
	 * Implementation from operator 3 in
	 * <a href="https://arxiv.org/abs/1811.07049">RMPFlow Section 3.4</a>.
	 * 
	 * @return A the desired acceleration which is a(x, x_dot)
	 */
	public SimpleMatrix getA() {
		solver.setA(m.getDDRM());
		DMatrixRMaj a = new DMatrixRMaj(m.getDDRM().getNumRows(), 1);
		solver.solve(f.getDDRM(), a);
		return new SimpleMatrix(a);
	} // TODO: Check for exception if inversion fails and return entire RMP tree in
		// exception throw
}
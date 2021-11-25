package com.titanrobotics2022.motion.generation.rmpflow;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

/**
 * A wrapper class that allows Lambda function to be used with RMPFlow
 */
public class RMP extends RMPLeaf {
    private Function<SimpleMatrix, SimpleMatrix> psi, j;
    private BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> j_dot;
    private Supplier<SimpleMatrix> f, m;

	public RMP(String name, RMPNode parent
				, Function<SimpleMatrix, SimpleMatrix> psi, Function<SimpleMatrix, SimpleMatrix> j
				, BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> j_dot
				, Supplier<SimpleMatrix> f, Supplier<SimpleMatrix> m)
    {
        super(name, parent);
        this.psi = psi;
        this.j = j;
        this.j_dot = j_dot;
    }

	@Override
	public SimpleMatrix psi(SimpleMatrix q)
	{
		return psi.apply(q);
	}

	@Override
	public SimpleMatrix j(SimpleMatrix q)
	{
		return j.apply(q);
	}

	@Override
	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot)
	{
		return j_dot.apply(q, q_dot);
    }
    
	@Override
    protected SimpleMatrix solveF(SimpleMatrix x, SimpleMatrix x_dot)
    {
        return f.get();
    }
	
	@Override
    protected SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot)
    {
        return m.get();
    }
}

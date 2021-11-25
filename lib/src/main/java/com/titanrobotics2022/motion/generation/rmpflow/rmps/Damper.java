package com.titanrobotics2022.motion.generation.rmpflow.rmps;

import com.titanrobotics2022.motion.generation.rmpflow.RMPLeaf;
import com.titanrobotics2022.motion.generation.rmpflow.RMPNode;

import org.ejml.simple.SimpleMatrix;

public class Damper extends RMPLeaf{
    private double w;
    private double eta;

    public Damper(String name, RMPNode parent, double eta, double w){
        super(name, parent);
        this.eta = eta;
        this.w = w;
    }
         
    public SimpleMatrix psi(SimpleMatrix q)
	{
		return q;
	}

	public SimpleMatrix j(SimpleMatrix q)
	{
		return SimpleMatrix.identity(2);
	}

	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot)
	{
		return new SimpleMatrix(2, 2);
	}


    public SimpleMatrix solveF(SimpleMatrix x, SimpleMatrix x_dot){
        SimpleMatrix bx_dot = x_dot.scale(-(eta * w));
        return bx_dot;
    }

    
    public SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot){
        return new SimpleMatrix(1, 1, false, new double[]{w});
    }

}

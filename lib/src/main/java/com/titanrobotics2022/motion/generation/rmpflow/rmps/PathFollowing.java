package com.titanrobotics2022.motion.generation.rmpflow.rmps;

import java.time.Instant;
import java.time.Duration;

public class PathFollowing extends RMPLeaf {
    
    private Path path;
    double v, P, I, A, B, K, h;
    private Instant start = Instant.now();

    // Compute desired vertical acceleration (PI loop)
        // P(c_dot - v) + I(c - d)
            // P: Tuning constant
            // c_dot: Current velocity along path
            // v: Desired velocity
            // I: Tuning constant
            // c: Current distance along path (progress)
            // d: Projected ideal c-value
    // Compute desired horizontal acceleration (PD loop)
        // Ay - B(y_dot)
            // A: Tuning constant
            // y: Current distance from path
            // B: Tuning constant
            // y_dot: Current velocity towards path
    // Compute weight matrix
        // Importance of these acceleration changes
        // Likely Hessian
            // Matrix of second partial derivatives
    // Pullback desired combined acceleration vector and weight matrix
    
    public PathFollowing(String name, RMPNode parent, Path path, double v, double P, double I, double A, double B, double K, double h) {
        super(name, parent);
        this.path = path;
        this.v = v;
        this.P = P;
        this.I = I;
        this.A = A;
        this.B = B;
        this.kFore = K * Math.sin(h * Math.PI / 2);
        this.kSide = K * Math.cos(h * Math.PI / 2);
    } 
    
    public SimpleMatrix solveF(SimpleMatrix x, SimpleMatrix x_dot) {
        
        double pos = new Point(x.get(0), x.get(1));

        double c = path.getProgress(pos);
        double theta = Math.atan2(x_dot.get(1), x_dot.get(0) - path.getRotation(c).getRadians());
        double c_dot = x_dot.norm() * Math.cos(theta);
        double d = v * Duration.between(start, Instant.now()).toMillis() / 1000;
        
        double y = path.getPos(d);
        double y_dot = x_dot.norm() * Math.sin(theta);

        Point task_a = new Point(P * (c_dot - v) + I * (c - d), A * y - B * y_dot).rotateBy(new Rotation2d(-theta));
        
        SimpleMatrix a = new SimpleMatrix(2, 1);
        a.set(task_a.getX());
        a.set(task_a.getY());

        return solveM(x, x_dot).dot(a);

    }

    public SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot) {
        
        double pos = new Point(x.get(0), x.get(1));

        double c = path.getProgress(pos);
        double theta = Math.atan2(x_dot.get(1), x_dot.get(0) - path.getRotation(c).getRadians());

        Point m = new Point(kFore, kSide).rotateBy(new Rotation2d(-theta));
        return new SimpleMatrix(new double[]{m.getX(), m.getY()});
        
    }

}

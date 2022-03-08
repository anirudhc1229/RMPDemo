package com.titanrobotics2022.motion.generation.rmpflow.rmps;

import java.time.Instant;

import com.titanrobotics2022.mapping.Path;
import com.titanrobotics2022.mapping.Point;
import com.titanrobotics2022.motion.generation.rmpflow.RMPLeaf;
import com.titanrobotics2022.motion.generation.rmpflow.RMPNode;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Rotation2d;

import java.time.Duration;

public class PathFollowing extends RMPLeaf {

    private Path path;
    private double v, P, I, A, B, kFore, kSide;
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
    // Matrix of second partial derivatives
    // Pullback desired combined acceleration vector and weight matrix

    public PathFollowing(String name, RMPNode parent, Path path, double v, double P, double I, double A, double B,
            double K, double h) {
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

        Point pos = new Point(x.get(0), x.get(1));

        double c = path.getProgress(pos);
        double theta = Math.atan2(x_dot.get(1), x_dot.get(0)) - path.getRotation(c).getRadians();
        double c_dot = x_dot.normF() * Math.cos(theta);
        double d = v * Duration.between(start, Instant.now()).toMillis() / 1000;

        double y = path.getPos(c).getDistance(pos);
        double y_dot = x_dot.normF() * Math.sin(theta);

        Point task_a = new Point(P * (v - c_dot) + I * (d - c), A * y - B * y_dot).rotateBy(new Rotation2d(-theta));

        SimpleMatrix a = new SimpleMatrix(2, 1);
        a.set(0, task_a.getX());
        a.set(1, task_a.getY());

        SimpleMatrix m = solveM(x, x_dot);
        SimpleMatrix f = m.mult(a);
        return f;

    }

    public SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot) {

        Point pos = new Point(x.get(0), x.get(1));

        double c = path.getProgress(pos);
        double theta = Math.atan2(x_dot.get(1), x_dot.get(0)) - path.getRotation(c).getRadians();

        Point m = new Point(kFore, kSide).rotateBy(new Rotation2d(-theta));
        return new SimpleMatrix(2, 2, true, new double[] { m.getX(), 0, 0, m.getY() });

    }

    @Override
    public SimpleMatrix j(SimpleMatrix q) {
        Point pos = new Point(q.get(0), q.get(1));
        double c = path.getProgress(pos);
        Rotation2d theta = path.getRotation(c);
        Rotation2d phi = pos.minus(path.getPos(c)).getAngle();
        return new SimpleMatrix(2, 2, true,
                new double[] { theta.getCos(), theta.getSin(), phi.getCos(), phi.getSin() });
    }

    @Override
    public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot) {
        Point pos = new Point(q.get(0), q.get(1));
        double c = path.getProgress(pos);
        Rotation2d theta = path.getRotation(c);
        Rotation2d phi = pos.minus(path.getPos(c)).getAngle();
        double dcdq = theta.getCos() * q_dot.get(0) + theta.getSin() * q_dot.get(1);
        return new SimpleMatrix(2, 2, true,
                new double[] { -theta.getSin(), theta.getCos(), -phi.getSin(), phi.getCos() })
                .scale(dcdq * path.getAngularVelocity(c).getRadians());
    }

}

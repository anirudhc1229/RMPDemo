package com.titanrobotics2022.motion.generation.rmpflow.rmps;

import java.time.Instant;

import com.titanrobotics2022.mapping.Path;
import com.titanrobotics2022.mapping.Point;
import com.titanrobotics2022.motion.generation.rmpflow.RMPLeaf;
import com.titanrobotics2022.motion.generation.rmpflow.RMPNode;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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

    @Override
    public SimpleMatrix psi(SimpleMatrix x) {
        double c = path.getProgress(new Point(x.get(0), x.get(1)));
        double theta = path.getRotation(c).getDegrees();
        double s = Math.sin(theta - Math.atan2(path.getPos(c).getY() - x.get(1), path.getPos(c).getX() - x.get(0)));
        double d = path.getPos(c).getDistance(new Translation2d(x.get(0), x.get(1))) * Math.signum(s);
        return new SimpleMatrix(2, 1, true, new double[] { c, d });
    }

    @Override
    public SimpleMatrix solveF(SimpleMatrix x, SimpleMatrix x_dot) {
        double t = v * Duration.between(start, Instant.now()).toMillis() / 1000;
        SimpleMatrix a = new SimpleMatrix(2, 1, true,
                new double[] { P * (v - x_dot.get(0)) + I * (v * t - x.get(0)), A * x.get(1) + B * x_dot.get(1) });
        return solveM(x, x_dot).mult(a);
    }

    @Override
    public SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot) {
        return new SimpleMatrix(2, 2, true, new double[] { kFore, 0, 0, kSide });
    }

    @Override
    public SimpleMatrix j(SimpleMatrix x) {
        double c = path.getProgress(new Point(x.get(0), x.get(1)));
        double theta = path.getRotation(c).getDegrees();
        return new SimpleMatrix(2, 2, true,
                new double[] { Math.cos(theta), Math.sin(theta), -Math.sin(theta), Math.cos(theta) });
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

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
        double theta = path.getRotation(c).getRadians();
        double s = -Math.signum(
                Math.sin(theta - Math.atan2(path.getPos(c).getY() - x.get(1), path.getPos(c).getX() - x.get(0))));
        double d = path.getPos(c).getDistance(new Translation2d(x.get(0), x.get(1))) * s;
        return new SimpleMatrix(2, 1, true, new double[] { c, d });
    }

    @Override
    public SimpleMatrix solveF(SimpleMatrix x, SimpleMatrix x_dot) {
        System.out.println("getV: " + getV(x));
        System.out.println("getD: " + getD());
        SimpleMatrix a = new SimpleMatrix(2, 1, true,
                new double[] { P * (getV(x) - x_dot.get(0)) + I * (getD() - x.get(0)),
                        A * x.get(1) - B * x_dot.get(1) });
        return solveM(x, x_dot).mult(a);
    }

    @Override
    public SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot) {
        return new SimpleMatrix(2, 2, true, new double[] { kFore, 0, 0, kSide });
    }

    @Override
    public SimpleMatrix j(SimpleMatrix x) {
        double c = path.getProgress(new Point(x.get(0), x.get(1)));
        double theta = path.getRotation(c).getRadians();
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

    public double getV(SimpleMatrix x) {
        System.out.println("getV x0: " + x.get(0));
        System.out.println("getV x1: " + x.get(1));
        double distFromHalf = x.get(0) - 0.5 * path.getLength();
        System.out.println("dfh: " + distFromHalf);
        if (distFromHalf > 0)
            return Math.sqrt(Math.pow(v, 2) - (2 * Math.pow(v, 2) * distFromHalf) / path.getLength());
        return v;
    }

    public double getD() {
        double t = Duration.between(start, Instant.now()).toMillis() / 1000.0;
        System.out.println("t: " + t);
        double tAfterHalf = path.getLength() / (2 * v);
        if (t < tAfterHalf)
            return v * t;
        double d = (path.getLength() / 2) + (v * tAfterHalf) - ((Math.pow(v, 2) * tAfterHalf) / (2 * path.getLength()));
        return Math.min(d, path.getLength());
    }

}

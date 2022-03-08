package com.titanrobotics2022.mapping;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A line segment.
 */
public class LinearSegment implements Path {
    /** The start point of this line segment. */
    protected final Point start;
    /** The end point of this line segment. */
    protected final Point end;

    /**
     * Creates a new LinearSegment with the specified start and end points.
     * 
     * @param start The start point of the segment.
     * @param end   The end point of the segment.
     */
    public LinearSegment(Point start, Point end) {
        this.start = start;
        this.end = end;
    }

    @Override
    public double getLength() {
        return end.minus(start).getNorm();
    }

    @Override
    public Point getPos(double distance) {
        return start.plus(new Point(distance, end.minus(start).getAngle()));
    }

    @Override
    public Rotation2d getRotation(double distance) {
        return end.minus(start).getAngle();
    }

    @Override
    public Rotation2d getAngularVelocity(double distance) {
        return new Rotation2d(0);
    }

    @Override
    public LinearSegment translateBy(Translation2d offset) {
        return new LinearSegment(start.plus(offset), end.plus(offset));
    }

    @Override
    public LinearSegment rotateBy(Rotation2d rotation) {
        return new LinearSegment(start.rotateBy(rotation), end.rotateBy(rotation));
    }

    @Override
    public double getDistance(Point from) {
        Rotation2d theta = Point.getAngle(end, start, from);
        double startDist = start.getDistance(from);
        if (theta.getCos() >= 0 && theta.getCos() * startDist < getLength())
            return Math.abs(theta.getSin() * startDist);
        return Math.min(startDist, end.getDistance(from));
    }

    /**
     * Gets the nearest point of this segment to a specified point.
     * 
     * @param from The point to find the nearest point to.
     * @return The point along this segment that is nearest to the specified
     *         point.
     */
    public Point getNearest(Point from) {
        Rotation2d theta = Point.getAngle(end, start, from);
        if (theta.getCos() <= 0)
            return start;
        double r = theta.getCos() * start.getDistance(from);
        if (r >= getLength())
            return end;
        return start.plus(new Point(r, end.minus(start).getAngle()));
    }

    @Override
    public double getProgress(Point pos) {
        return Point.getAngle(end, start, pos).getCos() * pos.getDistance(start);
    }

    /**
     * Gets the start point of this segment.
     * 
     * @return The start point of this segment.
     */
    public Point getStart() {
        return this.start;
    }

    /**
     * Gets the end point of this segment.
     * 
     * @return The end point of this segment.
     */
    public Point getEnd() {
        return this.end;
    }

    /**
     * Determines whether this path intersects another path.
     * 
     * <p>
     * Implements {@link Path#intersects(Path)} for the case where this path
     * and the other path are both LinearSegments.
     * 
     * @param other The path to test for intersection with.
     * @return True, if the paths intersect, or false, otherwise.
     */
    public boolean intersects(LinearSegment other) {
        double abx, aby, xya, xyb;
        abx = Point.getAngle(start, end, other.start).getSin();
        aby = Point.getAngle(start, end, other.end).getSin();
        xya = Point.getAngle(other.start, other.end, start).getSin();
        xyb = Point.getAngle(other.start, other.end, end).getSin();
        if (abx * aby <= 0 && xya * xyb <= 0)
            return true;
        else
            return false;
    }

    /**
     * Gets the minimum distance between this path and another path.
     * 
     * <p>
     * Implements {@link Path#getDistance(Path)} for the case where this path
     * and the other path are both LinearSegments.
     * 
     * @param other The path to find the distance to.
     * @return The minimum distance between any pair point along this path with
     *         any point on the other path.
     */
    public double getDistance(LinearSegment other) {
        if (intersects(other))
            return 0;
        else
            return Math.min(getDistance(other.start), getDistance(other.end));
    }

    @Override
    public LinearSegment reverse() {
        return new LinearSegment(end, start);
    }

    @Override
    public double getDistance(Path other) {
        if (other instanceof LinearSegment)
            return getDistance((LinearSegment) other);
        else
            return other.getDistance((LinearSegment) this);
    }
}
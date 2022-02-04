package com.titanrobotics2022.mapping;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * A path along a plane.
 */
public interface Path {
    /** Returns the length of this path. */
    public double getLength();

    /**
     * Gets a position along this path.
     * 
     * @param distance  The along this path.
     * @return The position at the specified distance along this path.
     */
    public Point getPos(double distance);

    /**
     * Returns the starting position of this path.
     * 
     * <p>This is equivalent to {@code getPos(0)};
     * 
     * @see #getPos(double)
     */
	default public Point getStart() {
        return getPos(0);
    }

    /** Returns the ending position of this path.
     * 
     * <p>This is equivalent to {@code getPos(getLength())};
     * 
     * @see #getPos(double)
     * @see #getLength()
     */
	default public Point getEnd() {
        return getPos(getLength());
    }

    /**
     * Gets the rotation at a specified distance along this path.
     * 
     * @param distance  The distance along this path.
     * @return The direction of motion along this path at the specified distance
     *  along this path.
     */
	public Rotation2d getRotation(double distance);
    
    /**
     * Gets the velocity at a specified distance along this path.
     * 
     * @param distance  The distance along this path.
     * @param speed  The speed of motion along this path.
     * @return The velocity of motion along this path at the specified distance
     *  along this path.
     */
    default public Point getVelocity(double distance, double speed) {
        return getVelocity(distance).times(speed);
    }
    /**
     * Gets the velocity at a specified distance along this path.
     * 
     * <p>This is equivanelt to {@link #getVelocity(double, double)} with a
     * speed of 1.
     * 
     * @param distance  The distance along this path.
     * @param speed  The speed of motion along this path.
     * @return The velocity of motion along this path at the specified distance
     *  along this path. Equivalently, this can be viewed as the derivative of
     *  the position along this path with respect to the distance along this
     *  path, evaluated at the specified point.
     */
    default public Point getVelocity(double distance) {
        return new Point(1, getRotation(distance));
    }
	
    /**
     * Gets the angular velocity at a specified distance along this path.
     * 
     * @param distance  The distance along this path.
     * @param speed  The speed of motion along this path.
     * @return The angular velocity of a particle moving along this path at the
     *  specified distance with the supplied velocity, assuming the particle is
     *  always oriented such that it is moving "forward".
     */
    default public Rotation2d getAngularVelocity(double distance, double speed) {
        return getAngularVelocity(distance).times(speed);
    }
    /**
     * Gets the velocity at a specified distance along this path.
     * 
     * <p>This is equivanelt to {@link #getAngularVelocity(double, double)} with
     * a speed of 1.
     * 
     * @param distance  The distance along this path.
     * @return The angular velocity of a particle moving along this path at the
     *  specified distance with the supplied velocity, assuming the particle is
     *  always oriented such that it is moving "forward". Equivalently, this can
     *  be viewed as the derivative of the orientation of such a particle with
     *  respect to the distance along this path, evaluated at the specified
     *  point.
     */
    public Rotation2d getAngularVelocity(double distance);
    
    /**
     * Gets the acceleration at a specified distance along this path.
     * 
     * @param distance  The distance along this path to evaluate the
     *  acceleration at.
     * @param speed  The speed of motion along this path.
     * @param acceleration  The (magnitude of the forward) acceleration along
     *  this path. That is, the acceleration ignoring centripital acceleration.
     * @return The acceleration of a particle moving along this path at the
     *  specified distance, with the provided speed and (non-cintripital)
     *  acceleration.
     */
	default public Point getAcceleration(double distance, double speed, double acceleration) {
        return getAcceleration(distance, speed)
                .plus(new Point(acceleration, getRotation(distance)));
    }
    /**
     * Gets the acceleration at a specified distance along this path.
     * 
     * <p>This method is equivalent to
     * {@link #getAcceleration(double, double, double)} with an acceleration of
     * 0.
     * 
     * @param distance  The distance along this path to evaluate the
     *  acceleration at.
     * @param speed  The speed of motion along this path.
     * @return The acceleration of a particle moving along this path at the
     *  specified distance, with the provided speed. The returned value ignores
     *  acceleration in the direction of the path.
     */
    default public Point getAcceleration(double distance, double speed) {
        return getAcceleration(distance).times(speed);
    }
    /**
     * Gets the acceleration at a specified distance along this path.
     * 
     * <p>This method is equivalent to {@link #getAcceleration(double, double)}
     * with a speed of 1
     * 
     * @param distance  The distance along this path to evaluate the
     *  acceleration at.
     * @return The acceleration of a particle moving along this path at the
     *  specified distance, ignoreing acceleration in the direction of the path.
     *  Equivalently, this is the second derivative of the position of such a
     *  particle with respect to the distance along this path.
     */
    default public Point getAcceleration(double distance){
        return new Point(getAngularVelocity(distance).getRadians(),
                getRotation(distance).plus(new Rotation2d(Math.PI / 2)));
    }
	
	/**
     * Applies a translation to this path.
     * 
     * @param offset  The translation to apply.
     * @return The translated path.
     */
	public Path translateBy(Translation2d offset);
	
	/**
     * Applies a rotation to this path.
     * 
     * @param rotation  The rotation to apply.
     * @return The rotated path.
     */
    public Path rotateBy(Rotation2d angle);
    
	/**
     * Reverses the direction of this path.
     * 
     * @return The reversed path.
     */
    public Path reverse();
	
	/**
     * Gets the minimum distance from this path to a point.
     * 
     * @param from  The point to find the distance to.
     * @return The minimum distance between any point along this path and the
     *  specified point.
     */
    public double getDistance(Point from);
    
	/**
     * Gets the minimum distance between this path and another path.
     * 
     * <p>The default implementation of this method calls the
     * {@link #getDistance(Path)} method of the other path with this path as the
     * argument. If this results in a StackOverflowError (for instance, if both
     * paths are using this default implementation), an
     * UnsupportedOperationException is thrown. Implementations of the Path
     * interface are strongly encouraged to implement this method or overload it
     * for as many cases as possible.
     * 
     * @param other  The path to find the distance to.
     * @return The minimum distance between any pair point along this path with
     *  any point on the other path.
     */
	default public double getDistance(Path other) {
        try{
            return other.getDistance(this);
        }
        catch(StackOverflowError err){
            throw new UnsupportedOperationException(err);
        }
    }

    public Point getNearest(Point from);

    public double getProgress(Point pos);

    /**
     * Determines whether this path intersects another path.
     * 
     * <p>This is equivalent to testing whether {@link #getDistance(other)}
     * with the desired path is 0. This means that paths which are angent to
     * each other, but do not cross, are considered to intersect by this method.
     * 
     * @param other  The path to test for intersections with.
     * @return True, if the paths intersect, or false, otherwise.
     */
	default public boolean intersects(Path other) {
        return getDistance(other) == 0;
    }
}
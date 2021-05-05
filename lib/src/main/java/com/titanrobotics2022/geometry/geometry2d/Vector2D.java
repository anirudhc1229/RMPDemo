package com.titanrobotics2022.geometry.geometry2d;

import org.apache.commons.math3.util.FastMath;

/**
 * A vector 2D class
 */
public class Vector2D {
    /** Origin (coordinates: 0, 0). */
    public static final Vector2D ZERO = new Vector2D(0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final Vector2D NAN = new Vector2D(Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final Vector2D POSITIVE_INFINITY = new Vector2D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final Vector2D NEGATIVE_INFINITY = new Vector2D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
    
    /** The x component. */
    public final double x;

    /** The y componenet. */
    public final double y;

    /**
     * Creates a cartesian vector. 
     * @param x component
     * @param y component
     */
    public Vector2D(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    /**
     * Adds two vectors together
     * @param rhs the vector to be added
     * @return the resulting vector
     */
    public Vector2D plus(Vector2D rhs)
    {
        return new Vector2D(this.x + rhs.x, this.y + rhs.y);
    }

    /**
     * Subtracts two vectors
     * @param rhs rhs the vector to be subtracted
     * @return the resulting vector
     */
    public Vector2D minus(Vector2D rhs)
    {
        return new Vector2D(this.x - rhs.x, this.y - rhs.y);
    }

    /**
     * 
     * @param scalar
     * @return
     */
    public Vector2D scalarMultiply(double scalar)
    {
        return new Vector2D(this.x * scalar, this.y * scalar);
    }

    /**
     * 
     * @param divisor
     * @return
     */
    public Vector2D scalarDivide(double divisor)
    {
        return new Vector2D(this.x / divisor, this.y / divisor);
    }

    /**
     * 
     * @return
     */
    public Vector2D negate()
    {
        return new Vector2D(-x, -y);
    }

    /**
     * 
     * @return
     */
    public double magnitude()
    {
        return FastMath.sqrt(x * x + y * y);
    }

    /**
     * 
     * @return
     */
    public double magnitudeSquared()
    {
        return x * x + y * y;
    }


    /**
     * The azimuthal angle is the angle from the x axis in the x-y plane (phi in physics, theta in math)
     * @return
     */
    public double azimuthalAngle()
    {
        double answer = FastMath.atan2(y, x);
        if (answer < 0)
            answer += 2 * Math.PI;
        return answer;
    }

    /**
     * 
     * @return
     */
    public Vector2D unitVector()
    {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
        {
            return ZERO;
        }
        else
        {
            return scalarDivide(FastMath.sqrt(mag2));
        }
    }

    /**
     * 
     * @param rhs
     * @return
     */
    public double dot(Vector2D rhs)
    {
        return x * rhs.x + y * rhs.y;
    }

    /**
     * 
     * @param rhs
     * @return
     */
    public double cross(Vector2D rhs)
    {
        return x * rhs.y - rhs.x * y;
    }

    /**
     * 
     * @param otherVec
     * @return
     */
    public Vector2D projectOnto(Vector2D otherVec)
    {
        Vector2D unitVec = otherVec.unitVector();
        return unitVec.scalarMultiply(dot(unitVec));
    }

    /**
     * 
     * @param v1
     * @param v2
     * @return
     */
    public static double angleBetween(Vector2D v1, Vector2D v2)
    {
        return FastMath.acos(v1.dot(v2) / (v1.magnitude() * v2.magnitude()));
    }

    public static Vector2D polarVector(double r, double phi)
    {
        double x = r * FastMath.cos(phi);
        double y = r * FastMath.sin(phi);
        return new Vector2D(x, y);
    }

    /**
     * 
     */
    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof Vector2D) {
            final Vector2D rhs = (Vector2D) obj;
            return (x == rhs.x) && (y == rhs.y);
        }
        return false;
    }

    /**
     * 
     * @param rhs
     * @param tolerance
     * @return
     */
    public boolean equals(Vector2D rhs, double tolerance)
    {
        return FastMath.abs(x - rhs.x) < tolerance && FastMath.abs(y - rhs.y) < tolerance;
    }
}

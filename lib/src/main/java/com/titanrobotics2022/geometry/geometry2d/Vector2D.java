package com.titanrobotics2022.geometry.geometry2d;

/**
 * A vector 2D class
 */
public class Vector2D {
    public final double x, y;

    /**
     * Creates a cartesian vector 
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
}

package com.titanrobotics2022.mapping;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * A point on a 2D cartesian plane.
 */
public class Point extends Translation2d {
  
  /**
   * Creates a point with specified cartesian coordinates.
   * 
   * @param x  The x coordinate of this point.
   * @param y  The y coordinate of this point.
   */
  public Point(double x, double y){
    super(x, y);
  }
  /**
   * Creates a point from polar coordinates
   * 
   * @param r  The distance from the origin to the point.
   * @param theta  The angle from the positive x-axis to the vector from the
   *  origin to the point.
   */
  public Point(double r, Rotation2d theta) {
    this(theta.getCos() * r, theta.getSin() * r);
  }
  /** Creates a point from a Translation2d.
   * 
   * @param translation  The translation to create this Point from.
   */
  public Point(Translation2d translation) {
    this(translation.getX(), translation.getY());
  }
  
  /**
   * Computes the angle between three points.
   * 
   * @param base  The point defining the ray the angle is measured from.
   * @param vertex  The point defining the common endpoint of the rays
   *  forming the angle to measure.
   * @param leg  The point defining the ray the angle is measured to.
   * @return  The signed angle from the ray from {@code vertex} to {@code base}
   *  to the ray from {@code vertex} to {@code leg}.
   */
  public static Rotation2d getAngle(Point base, Point vertex, Point leg) {
    return leg.minus(vertex).getAngle().minus(base.minus(vertex).getAngle());
  }

  /** Returns the polar angle of this Point. */
  public Rotation2d getAngle() {
    return new Rotation2d(getX(), getY());
  }

  @Override
  public Point plus(Translation2d other) {
    return new Point(getX() + other.getX(), getY() + other.getY());
  }

  @Override
  public Point minus(Translation2d other) {
    return new Point(getX() - other.getX(), getY() - other.getY());
  }

  @Override
  public Point unaryMinus() {
    return new Point(-getX(), -getY());
  }

  @Override
  public Point rotateBy(Rotation2d other) {
    return new Point(getX()*other.getCos() - getY()*other.getSin(),
        getX()*other.getSin() + getY()*other.getCos());
  }

  @Override
  public Point times(double scalar) {
    return new Point(getX()*scalar, getY()*scalar);
  }

  @Override
  public Point div(double scalar) {
    return new Point(getX()/scalar, getY()/scalar);
  }

  @Override
  public String toString() {
    return String.format("Point(%.2f, %.2f)", getX(), getY());
  }

  @Override
  public boolean equals(Object o) {
    if(o instanceof Translation2d)
      return getDistance((Translation2d) o) < 0.00001;
    return super.equals(o);
  }
}

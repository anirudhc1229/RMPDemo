package com.titanrobotics2022.geometry.geometry2d;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Vector;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.math3.util.FastMath;

public class Vector2DTest {
    private static final double delta = 1e-9;

    @Test
    void additionTest() {// TODO: edit to incorporate double precision error
        Vector2D a = new Vector2D(1, 1);
        Vector2D b = new Vector2D(2, 3);
        Vector2D actual = a.plus(b);
        assertEquals(1 + 2, actual.x, delta);
        assertEquals(1 + 3, actual.y, delta);
    }

    @Test
    void subtractionTest() {// TODO: edit to incorporate double precision error
        Vector2D a = new Vector2D(1, 1);
        Vector2D b = new Vector2D(2, 3);
        Vector2D actual = a.minus(b);
        assertEquals(1 - 2, actual.x, delta);
        assertEquals(1 - 3, actual.y, delta);
    }

    @Test
    void scalarMultiplicationTest() {// TODO: edit to incorporate double precision error
        final double scalar = 2.4;
        Vector2D a = new Vector2D(2.5, 3.5);
        Vector2D actual = a.scalarMultiply(scalar);
        assertEquals(2.5 * scalar, actual.x, delta);
        assertEquals(3.5 * scalar, actual.y, delta);
    }

    @Test
    void scalarDivisionTest() {// TODO: edit to incorporate double precision error
        final double divisor = 2.4;
        Vector2D a = new Vector2D(2.5, 3.5);
        Vector2D actual = a.scalarDivide(divisor);
        assertEquals(2.5 / divisor, actual.x, delta);
        assertEquals(3.5 / divisor, actual.y, delta);
    }

    @Test
    void negationTest() {
        Vector2D a = new Vector2D(1, -1);
        Vector2D actual = a.negate();
        assertEquals(-1, actual.x);
        assertEquals(1, actual.y);
    }

    @Test
    void magnitudeTest() {
        Vector2D a = new Vector2D(2, 2);
        double actual = a.magnitude();
        assertEquals(Math.sqrt(2 * 2 + 2 * 2), actual, delta);
    }

    @Test
    void magnitudeSquaredTest() {
        Vector2D a = new Vector2D(2, 2);
        double actual = a.magnitudeSquared();
        assertEquals(2 * 2 + 2 * 2, actual, delta);
    }

    @Test
    void azimuthalAngleTest() {
        Vector2D positiveX = new Vector2D(2, 2);
        Vector2D negativeX = new Vector2D(-2, 2);
        Vector2D alongPositiveX = new Vector2D(2, 0);
        double actual = positiveX.azimuthalAngle();
        assertEquals(Math.PI / 4.0, actual, delta);
        actual = negativeX.azimuthalAngle();
        assertEquals(3 * Math.PI / 4.0, actual, delta);
        actual = alongPositiveX.azimuthalAngle();
        assertEquals(0, actual, delta);
    }

    @Test
    void unitVectorTest() {
        Vector2D zeroVector = new Vector2D(0, 0);
        Vector2D a = new Vector2D(-2, 2);
        Vector2D actual = zeroVector.unitVector();
        assertTrue(Vector2D.ZERO == actual); // Comparing object references
        actual = a.unitVector();
        double magnitude = FastMath.sqrt(-2 * -2 + 2 * 2);
        Vector2D expected = new Vector2D(-2 / magnitude, 2 / magnitude);
        assertTrue(actual.equals(expected, delta));
    }

    @Test
    void dotProductTest() {
        Vector2D a = new Vector2D(2, 3);
        Vector2D b = new Vector2D(-3, 3);
        double actual = a.dot(b);
        assertEquals(2 * -3 + 3 * 3, actual, delta);
    }

    @Test
    void crossProductTest() {
        Vector2D a = new Vector2D(2, 3);
        Vector2D b = new Vector2D(-3, 3);
        double actual = a.cross(b);
        assertEquals(2 * 3 - (-3 * 3), actual, delta);
    }

    @Test
    void projectionTest() // TODO: Implement
    {
        Vector2D a = new Vector2D(1, 1);
        Vector2D b = new Vector2D(2, 0);
        Vector2D longerB = new Vector2D(10, 0);
        Vector2D actual = a.projectOnto(b);
        assertEquals(1, actual.x, delta);
        assertEquals(0, actual.y, delta);
        actual = a.projectOnto(longerB);
        assertEquals(1, actual.x, delta);
        assertEquals(0, actual.y, delta);
    }

    @Test
    void angleBetweenTest() // TODO: Implement also
    {
        Vector2D a = new Vector2D(1, 1);
        Vector2D b = new Vector2D(1, 0);
        Vector2D c = new Vector2D(0, -1);
        double actual = Vector2D.angleBetween(a, b);
        assertEquals(Math.PI / 4, actual, delta);
        actual = Vector2D.angleBetween(b, a);
        assertEquals(Math.PI / 4, actual, delta);
        actual = Vector2D.angleBetween(a, b.negate());
        assertEquals(3 * Math.PI / 4, actual, delta);
        actual = Vector2D.angleBetween(a, c);
        assertEquals(3 * Math.PI / 4, actual, delta);
    }

    @Test
    void equalsTest() {
        final double x = 1, y = 2;
        Vector2D ones = new Vector2D(x, x);
        Vector2D ones2 = new Vector2D(x, x);
        Vector2D twoes = new Vector2D(y, y);
        assertEquals(true, ones.equals(ones2));
        assertEquals(false, ones.equals(twoes));
    }

    @Test
    void equalsWithToleranceTest() {
        final double x = 1.55, y = 2.55;
        Vector2D ones = new Vector2D(x, x);
        Vector2D ones2 = new Vector2D(x, x);
        Vector2D twoes = new Vector2D(y, y);
        assertEquals(true, ones.equals(ones2, delta));
        assertEquals(false, ones.equals(twoes, delta));
    }
}

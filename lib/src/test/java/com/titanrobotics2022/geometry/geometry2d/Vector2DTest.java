package com.titanrobotics2022.geometry.geometry2d;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class Vector2DTest {
    @Test
    public void additionTest() {// TODO: edit to incorporate double precision error
        Vector2D a = new Vector2D(1, 1);
        Vector2D b = new Vector2D(2, 3);
        Vector2D actual = a.plus(b);
        assertTrue(actual.x == 1 + 2);
        assertTrue(actual.y == 1 + 3);
    }
}

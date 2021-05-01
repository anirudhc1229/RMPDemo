package com.titanrobotics2022.geometry.geometry3d;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class Vector3DTest {
    @Test
    public void additionTest() {// TODO: edit to incorporate double precision error
        Vector3D a = new Vector3D(1, 1, 1);
        Vector3D b = new Vector3D(2, 3, 4);
        Vector3D actual = a.plus(b);
        assertTrue(actual.x == 1 + 2);
        assertTrue(actual.y == 1 + 3);
        assertTrue(actual.z == 1 + 4);
    }
}

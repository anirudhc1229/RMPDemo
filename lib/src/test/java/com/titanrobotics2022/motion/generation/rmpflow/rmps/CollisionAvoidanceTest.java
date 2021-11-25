package frc.robot.motion.generation.rmpflow;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
    RMPRoot r = new RMPRoot("root");
    @Test
    void ConvertCenterToColumnMartixTest()
    {
        CollisionAvoidance policy = new CollisionAvoidance("Collision Avoidance Test", r, new SimpleMatrix(1, 2,  false, new double[] {0, 0}), 1.0, .2, 1e-5, 0.0);
        assertTrue(policy.getCenter().numCols() == 1);
    }
}
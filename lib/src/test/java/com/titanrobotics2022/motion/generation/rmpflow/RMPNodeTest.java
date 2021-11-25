package frc.robot.motion.generation.rmpflow;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RMPNodeTest {
    RMPNode parent1;
    RMPNode parent2;
    RMPNode policy1;
    RMPNode policy2;

    @BeforeEach
    void createTwoLayerPopulatedRootTree()
    {
        parent1 = new RMPRoot("parent 1");
        parent2 = new RMPRoot("parent 2");
        policy1 = new CollisionAvoidance("Collision Avoidance Policy 1", parent1, new SimpleMatrix(1, 2,  false, new double[] {0, 0}), 1.0, .2, 1e-5, 0.0);
        policy2 = new CollisionAvoidance("Collision Avoidance Policy 2", parent1, new SimpleMatrix(1, 2,  false, new double[] {0, 0}), 1.0, .2, 1e-5, 0.0);
    }
    @Test
    void linkParentTest()
    {
        policy1.linkParent(parent2);
        assertEquals(1, parent1.getChildren().size());
        assertEquals(parent2, policy1.getParent());// Object references
        boolean foundChild = false;
        for (var child : parent2.getChildren()) {
            if(child == policy1) foundChild = true;
        }
        assertTrue(foundChild);
    }
    @Test
    void unlinkParentTest()
    {
        policy1.unlinkParent(parent1);
        assertEquals(1, parent1.getChildren().size());
        assertEquals(null, policy1.getParent());// Object references
        boolean foundChild = false;
        for (var child : parent2.getChildren()) {
            if(child == policy1) foundChild = true;
        }
        assertTrue(!foundChild);
    }

    @Test
    void linkChildTest()
    {
        parent2.linkChild(policy1);
        assertEquals(1, parent2.getChildren().size());
        assertEquals(parent2, policy1.getParent());
        boolean foundChild = false;
        for (var child : parent2.getChildren()) {
            if (child == policy1) foundChild = true;
        }
        assertTrue(foundChild);
    }

    @Test
    void unlinkChildTest()
    {
        parent1.unlinkChild(policy2);
        assertEquals(1, parent1.getChildren().size());
        assertEquals(null, policy2.getParent());
        boolean foundChild = false;
        for (var child : parent2.getChildren()) {
            if (child == policy2) foundChild = true;
        }
        assertTrue(!foundChild);
    }

}

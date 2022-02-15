package com.titanrobotics2022.demos;

import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;

public class RunDemo {
    public static void main(String[] args) {
        RMPRoot root = new RMPRoot("root");
        PathFollowingDemo pfd = new PathFollowingDemo();
    }
}

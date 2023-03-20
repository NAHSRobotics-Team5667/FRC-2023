// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class FlatSurfaceFinder extends SubsystemBase {
    public static boolean isBlue = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue); // TODO: Check if
                                                                                                    // this works

    public static final class FieldConstants {
        public static final Rotation2d REDANGLE_ROTATION2D = new Rotation2d(0);
        public static final List<Pose2d> REDFLATSURFACES_POSE2DS = new ArrayList<Pose2d>() {
            {
                add(new Pose2d(0, 0, REDANGLE_ROTATION2D));
                add(new Pose2d(0, 0, REDANGLE_ROTATION2D));
                add(new Pose2d(0, 0, REDANGLE_ROTATION2D));
                add(new Pose2d(0, 0, REDANGLE_ROTATION2D));
                add(new Pose2d(0, 0, REDANGLE_ROTATION2D));
                add(new Pose2d(0, 0, REDANGLE_ROTATION2D));
                add(new Pose2d(0, 0, REDANGLE_ROTATION2D));
                add(new Pose2d(0, 0, REDANGLE_ROTATION2D));
                add(new Pose2d(0, 0, REDANGLE_ROTATION2D));

            }
        };
        public static final Rotation2d BLUE_ROTATION2D = new Rotation2d(0);
        public static final List<Pose2d> BLUEFLATSURFACES_POSE2DS = new ArrayList<Pose2d>() {
            {

                add(new Pose2d(0, 0, BLUE_ROTATION2D));
                add(new Pose2d(0, 0, BLUE_ROTATION2D));
                add(new Pose2d(0, 0, BLUE_ROTATION2D));
                add(new Pose2d(0, 0, BLUE_ROTATION2D));
                add(new Pose2d(0, 0, BLUE_ROTATION2D));
                add(new Pose2d(0, 0, BLUE_ROTATION2D));
                add(new Pose2d(0, 0, BLUE_ROTATION2D));
                add(new Pose2d(0, 0, BLUE_ROTATION2D));
                add(new Pose2d(0, 0, BLUE_ROTATION2D));

            }
        };
    }

    /** Creates a new PoleFinder. */
    public FlatSurfaceFinder() {
    }

    public static Pose2d getNearestPole() {
        return RobotContainer.poseEstimate.getCurrentPose()
                .nearest(isBlue ? FieldConstants.BLUEFLATSURFACES_POSE2DS : FieldConstants.REDFLATSURFACES_POSE2DS);
    }

    @Override
    public void periodic() {
    }
}

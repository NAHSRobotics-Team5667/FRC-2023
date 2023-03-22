// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GetSticksMode;
import java.util.function.BooleanSupplier;

import frc.robot.util.FlatSurfaceFinder;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignFlatSurface extends ParallelRaceGroup {
    PathPlannerTrajectory FlatSurfaceLocation;
    BooleanSupplier getSticks;

    /** Creates a new AlignFlatSurfaceAgain. */
    public AlignFlatSurface(RobotContainer robotContainer) {

        PathPlannerTrajectory FlatSurfaceLocation = PathPlanner.generatePath(new PathConstraints(5, 5),
                new PathPoint(
                        new Translation2d(RobotContainer.poseEstimate.getCurrentPose().getX(),
                                RobotContainer.poseEstimate.getCurrentPose().getY()),
                        RobotContainer.poseEstimate.getCurrentPose().getRotation()),
                new PathPoint(
                        new Translation2d(FlatSurfaceFinder.getNearestPole().getX(),
                                FlatSurfaceFinder.getNearestPole().getY()),
                        FlatSurfaceFinder.getNearestPole().getRotation()));
        this.FlatSurfaceLocation = FlatSurfaceLocation;
        BooleanSupplier getSticks = robotContainer.getSticks(GetSticksMode.SURFACE);

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(robotContainer.autoBuilder.fullAuto(FlatSurfaceLocation));
        until(getSticks);
    }
}

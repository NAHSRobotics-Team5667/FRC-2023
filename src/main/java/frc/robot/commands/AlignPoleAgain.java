// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.RobotContainer.StickMode;
import java.util.function.BooleanSupplier;
import frc.robot.util.PoleFinder;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignPoleAgain extends ParallelRaceGroup {
    PathPlannerTrajectory FlatSurfaceLocation;
    BooleanSupplier getSticks;

    /** Creates a new AlignPoleAgain. */
    public AlignPoleAgain(RobotContainer m_RobotContainer) {
        this.FlatSurfaceLocation = PathPlanner.generatePath(
                new PathConstraints(5, 5),
                new PathPoint(new Translation2d(
                        RobotContainer.poseEstimate.getCurrentPose().getX(),
                        RobotContainer.poseEstimate.getCurrentPose().getY()),
                        RobotContainer.poseEstimate.getCurrentPose().getRotation()),
                new PathPoint(new Translation2d(
                        PoleFinder.getNearestPole().getX(),
                        PoleFinder.getNearestPole().getY()),
                        PoleFinder.getNearestPole().getRotation()));
        BooleanSupplier getSticks = m_RobotContainer.getState(StickMode.POLE);

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(m_RobotContainer.autoBuilder.fullAuto(FlatSurfaceLocation));
        until(getSticks);
    }
}

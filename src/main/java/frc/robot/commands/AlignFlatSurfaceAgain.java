// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;

import frc.robot.util.FlatSurfaceFinder;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignFlatSurfaceAgain extends ParallelRaceGroup {
    PathPlannerTrajectory FlatSurfaceLocation;
    BooleanSupplier getSticks;

    /** Creates a new AlignFlatSurfaceAgain.
    * @param m_RobotContainer The robot container
    */ // TODO: Same deal hear as line 42. you define the exact same thing twice, might as well not do that
    public AlignFlatSurfaceAgain(RobotContainer m_RobotContainer) {
        this.FlatSurfaceLocation = PathPlanner.generatePath(
            new PathConstraints( 5, 5), 
            new PathPoint(new Translation2d(
                RobotContainer.poseEstimate.getCurrentPose().getX(), 
                RobotContainer.poseEstimate.getCurrentPose().getY()), 
                RobotContainer.poseEstimate.getCurrentPose().getRotation()),
            new PathPoint(new Translation2d(
                FlatSurfaceFinder.getNearestPole().getX(), 
                FlatSurfaceFinder.getNearestPole().getY()), 
                FlatSurfaceFinder.getNearestPole().getRotation()));

        // TODO: consider putting this in robot container or a seperate class so you don't define it twice with AlignPoleAgain and even IntakeAndOuttakeProcedure
        BooleanSupplier getSticks = new BooleanSupplier() {
            public boolean getAsBoolean() {
                return // TODO: can these controller values be negative? if so then add math.abs or whatever.
                    ((MathUtil.applyDeadband(-RobotContainer.m_controller.getLeftX(), 0.1)) > 0) || 
                    ((MathUtil.applyDeadband(-RobotContainer.m_controller.getLeftY(), 0.1)) > 0) || 
                    Math.pow((Math.pow(FlatSurfaceFinder.getNearestPole().getX(), 2) + Math.pow(FlatSurfaceFinder.getNearestPole().getY(), 2)), .5) < .08; // Distance formula *Clap Clap*
            }
        };
        
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(m_RobotContainer.autoBuilder.fullAuto(FlatSurfaceLocation));
        until(getSticks);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.util.FlatSurfaceFinder;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignFlatSurfaceAgain extends ParallelRaceGroup {
  PathPlannerTrajectory FlatSurfaceLocation;
  private RobotContainer m_RobotContainer;
  private XboxController m_Controller;
  BooleanSupplier getSticks;

  
  /** Creates a new AlignFlatSurfaceAgain. */
  public AlignFlatSurfaceAgain() {
          


    this.m_Controller = m_RobotContainer.m_controller;
    PathPlannerTrajectory FlatSurfaceLocation = PathPlanner.generatePath(new PathConstraints( 5, 5), 
    new PathPoint(new Translation2d(RobotContainer.poseEstimate.getCurrentPose().getX(), RobotContainer.poseEstimate.getCurrentPose().getY()), RobotContainer.poseEstimate.getCurrentPose().getRotation()),
    new PathPoint(new Translation2d(FlatSurfaceFinder.getNearestPole().getX(), FlatSurfaceFinder.getNearestPole().getY()), FlatSurfaceFinder.getNearestPole().getRotation()));
    this.FlatSurfaceLocation = FlatSurfaceLocation;
    BooleanSupplier getSticks = new BooleanSupplier() {
      
      public boolean getAsBoolean() {
        // TODO Auto-generated method stub
        return ((MathUtil.applyDeadband(-RobotContainer.m_controller.getLeftX(), 0.1))> 0) || ((MathUtil.applyDeadband(-RobotContainer.m_controller.getLeftY(), 0.1))> 0) || Math.pow((Math.pow(FlatSurfaceFinder.getNearestPole().getX(), 2) + Math.pow(FlatSurfaceFinder.getNearestPole().getY(), 2)), .5) < .08;
      }
  };
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(m_RobotContainer.autoBuilder.fullAuto(FlatSurfaceLocation));
    until(getSticks);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.PoleFinder;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Translation2d;
public class AlignPole extends CommandBase {
  public LimelightSubsystem m_limelight;
  private RobotContainer m_RobotContainer;  
  /** Creates a new Align. */
  public AlignPole(LimelightSubsystem m_Limelight, RobotContainer robotContainer) {
    this.m_RobotContainer = robotContainer;
    this.m_limelight = m_Limelight;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
   

  }

  //Plan for align: first find if there are targets, pick the lowest one and center the robot to it. All heights for the robot will be based off of that. Afterwards we can add the ability to change height based off of distance, however we should initially just have code to do it from a set position

  //Might link this to a button press, like y or something. When its pressed if it sees a target within range it makes that the set point of the swerve drive MAKE SURE POSE IS FACING THE RIGHT WAY

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){

    PathPlannerTrajectory poleLocation = PathPlanner.generatePath(new PathConstraints( 5, 5), 
    new PathPoint(new Translation2d(RobotContainer.poseEstimate.getCurrentPose().getX(), RobotContainer.poseEstimate.getCurrentPose().getY()), RobotContainer.poseEstimate.getCurrentPose().getRotation()), 
    new PathPoint(new Translation2d(PoleFinder.getNearestPole().getX(), PoleFinder.getNearestPole().getY()), PoleFinder.getNearestPole().getRotation()));
    m_RobotContainer.autoBuilder.fullAuto(poleLocation);

    // if (RobotContainer.m_controller.getYButtonPressed() == true) {
    //   if (m_limelight.hasValidTarget()){
    //     if (m_limelight.getArea())
    //   }

    // }
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.pow((Math.pow(PoleFinder.getNearestPole().getX(), 2) + Math.pow(PoleFinder.getNearestPole().getY(), 2)), .5) < .08){
      return true;
    }
    return false;
  }
}

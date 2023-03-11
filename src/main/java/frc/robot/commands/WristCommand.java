// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
  public boolean isCube = false;
  WristSubsystem wrist;
  public static double[] Setpoints = { 
    0,
    0,
    0,
    0
    // these will be the heights of the slide at different points. The wrist angle will be set as Setpoints[bumperPos]
    
};
  /** Creates a new WristCommand. */
  public WristCommand(WristSubsystem wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setWrist(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int bumperPos = 0;
        if (RobotContainer.m_controller.getLeftBumperPressed()) {
            if (bumperPos == 3){
                bumperPos= 3;
            }
            else{
                bumperPos++;
            }
        }
        if (RobotContainer.m_controller.getRightBumperPressed()) {
            if (bumperPos == 0){
                bumperPos= 0;
            }
            else{
                bumperPos =- 1;
            }
        }
        if (RobotContainer.m_controller.getYButton()) {
            bumperPos = 0;
        }
        if (RobotContainer.m_controller.getXButton()){
            bumperPos = 3;
        }

    if (RobotContainer.m_controller.getRightBumperPressed()){
      wrist.cubeAngled(Setpoints[bumperPos]);
      isCube = true;
    }
    
    if (RobotContainer.m_controller.getLeftBumperPressed()){
      wrist.coneAngled();
      isCube = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

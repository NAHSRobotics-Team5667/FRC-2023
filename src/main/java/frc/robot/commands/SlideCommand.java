// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.SlideSubsystem;

public class SlideCommand extends CommandBase {
  private SlideSubsystem slide;
  /** Creates a new SlideCommand. */
  public SlideCommand(SlideSubsystem slide) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.slide = slide;
    addRequirements(slide);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    slide.setSlide(0);
    slide.setTilt(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // make or call a function to set slide to zero sets slide back to zero
    slide.setSlide(0);
    slide.setTilt(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

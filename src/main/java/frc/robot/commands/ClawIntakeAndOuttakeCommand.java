// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.util.CurrentSpikeCounter;

public class ClawIntakeAndOuttakeCommand extends CommandBase {
  public boolean intake;
  public ClawSubsystem clawSubsystem;
  public CurrentSpikeCounter spikeCounter;
  public boolean finished = false;

  /** Creates a new SlideIntakeAndOuttakeCommand. */
  public ClawIntakeAndOuttakeCommand( ClawSubsystem clawSubsystem, boolean intake) {
    this.intake = intake;
    this.clawSubsystem = clawSubsystem;

    
    addRequirements(clawSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake) {
      while (!ClawSubsystem.isPieceIntaken()){
        ClawSubsystem.setIntake(.2);
      }
      finished = true;

    }else{
      while (ClawSubsystem.isPieceIntaken()){
        ClawSubsystem.setIntake(-.2);
        
      }
      finished = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}

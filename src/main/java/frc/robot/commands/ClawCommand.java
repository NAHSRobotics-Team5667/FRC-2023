// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase {
    /** Creates a new ClawCommand. */
    public ClawCommand(ClawSubsystem claw) {
        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ClawSubsystem.setIntake(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ClawSubsystem.setIntake(.1);
   
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ClawSubsystem.setIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

//sup raf, hows spring break?

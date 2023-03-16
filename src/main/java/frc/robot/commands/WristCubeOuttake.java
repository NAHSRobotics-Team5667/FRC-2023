// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.RobotContainer;

public class WristCubeOuttake extends CommandBase {
    WristSubsystem wrist;
    RobotContainer robotContainer;

    /** Creates a new WristConeOuttake. */
    public WristCubeOuttake(WristSubsystem wrist, RobotContainer robotContainer) {
        this.wrist = wrist;
        this.robotContainer = robotContainer;

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        wrist.coneOuttakeAngled(); // might need driver control here, in case cone is in odd position
        if (Math.abs(wrist.pidError()) < .1) {
            robotContainer.outtakeFinish = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        robotContainer.outtakeFinish = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

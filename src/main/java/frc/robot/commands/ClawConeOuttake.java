// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;

public class ClawConeOuttake extends CommandBase {
    // TODO: this is a copy of ClawCubeOuttake, but with the intake speed reversed.
    // This is a bad way to do this. We should have a single command that takes a
    // parameter for the direction of the intake.
    // Counter TODO: The issue with making it all in one command is that the
    // ConfigureButtonBindings command only runs once, which means the triggers and
    // commands linked to them are only defined the way they are at instantiation,
    // meaning that we cant pass a changing variable in them to sense that.
    double stopClock;
    IntakeSubsystem claw;
    RobotContainer robotContainer;

    /** Creates a new IntakeOuttakeProcessClaw. */
    public ClawConeOuttake(IntakeSubsystem claw, RobotContainer robotContainer) {
        this.claw = claw;
        this.robotContainer = robotContainer;
        addRequirements(claw);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stopClock = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // runs for set time
        if (stopClock < 10) {
            claw.setIntake(-.45);
            stopClock += .02;
        } else {
            robotContainer.outtakeFinish = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        robotContainer.outtakeFinish = false;
        claw.setIntake(0);
        robotContainer.setPositionLevel(0);
        robotContainer.setCurrentElement(GamePiece.NONE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

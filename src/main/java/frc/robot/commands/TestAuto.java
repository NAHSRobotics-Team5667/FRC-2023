// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestAuto extends CommandBase {
    DrivetrainSubsystem drive;
    IntakeSubsystem claw;
    boolean isFinished = false;
    double timer = 0;

    /** Creates a new autoGoBrrrrr. */
    public TestAuto(DrivetrainSubsystem drive, IntakeSubsystem claw) {
        this.claw = claw;
        this.drive = drive;
        addRequirements(drive, claw);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer < 6) { // TODO: take a look at the timer in Lights.java:343 its a bit more concise to
                         // use the system clock rather than assume that periodic is called every 20ms
                         // Ok thats a good point, but like this makes sense. This is a placeholder auto
                         // until we start doing better with PathPlanner so I'm too lazy to change it

            claw.setIntake(1);
            timer += .02;
            if (timer < 6 && timer > 1) {
                this.drive.drive(0, 1, 0, false);
            }
        } else {
            isFinished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer = 0;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}

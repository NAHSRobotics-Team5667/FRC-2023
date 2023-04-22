// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.RobotContainer.GamePiece.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Lights;

public class ClawOuttake extends CommandBase {
    IntakeSubsystem claw;
    RobotContainer robotContainer;
    Lights lightstrip;
    GamePiece gamePiece;
    double delay = 0, clock = 0, intakeSpeed, stopClock;
    String welp;

    /** Creates a new IntakeOuttakeProcessClaw. */
    public ClawOuttake(GamePiece gamePiece, IntakeSubsystem claw, RobotContainer robotContainer, double delay) {
        this.gamePiece = gamePiece;
        this.claw = claw;
        this.robotContainer = robotContainer;
        this.lightstrip = robotContainer.lightstrip;
        this.delay = delay;

        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stopClock = 0;
        intakeSpeed = 0;
        clock = 0;
        welp = "huh";

        lightstrip.scheduler.setLightEffect(() -> {
            lightstrip.setSolidRGB(0, 255, 0);
        }, .6, 25, .14);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // runs for set time
        if (delay > clock && delay != 0) {
            clock += .02;
            welp = "stalled";
        } else {
            if (gamePiece == CUBE) {
                intakeSpeed = 0.4;
                welp = "cube going";
            } else if (gamePiece == CONE) {
                intakeSpeed = -.3;
                welp = "cone going";
            } else if (delay + 1 > clock && delay != 0) {
                intakeSpeed = 0;
                welp = "welp";
            }
            claw.setIntake(intakeSpeed);
        }

        SmartDashboard.putNumber("Clock", clock);
        SmartDashboard.putString("welp", welp);
        SmartDashboard.putBoolean("B Slide", RobotContainer.slideController.getBButton());
        SmartDashboard.putBoolean("A Slide", RobotContainer.slideController.getAButton());
        SmartDashboard.putBoolean("B Drive", RobotContainer.driveController.getBButton());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        robotContainer.outtakeFinish = false;
        claw.setIntake(0);
        robotContainer.setPositionLevel(0);
        robotContainer.setTargetElement(NONE);
        robotContainer.setCurrentElement(GamePiece.NONE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.RobotContainer.GamePiece.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.WristSubsystem;

public class ClawIntake extends CommandBase {
    // this is a copy of ClawConeIntake, with minor adjustments. It should be
    // refactored to a single command with a piece type parameter.
    private GamePiece gamePiece;
    public IntakeSubsystem clawSubsystem;
    public WristSubsystem wrist;
    public RobotContainer robotContainer;
    public Lights lightstrip;

    /** Creates a new SlideIntakeAndOuttakeCommand. */
    public ClawIntake(GamePiece gamePiece, IntakeSubsystem clawSubsystem, WristSubsystem wrist,
            RobotContainer robotContainer) {
        this.gamePiece = gamePiece;
        this.clawSubsystem = clawSubsystem;
        this.wrist = wrist;
        this.robotContainer = robotContainer;

        addRequirements(clawSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotContainer.setCurrentElement(GamePiece.NONE);
        robotContainer.setTargetElement(gamePiece);
        lightstrip = robotContainer.lightstrip;
        lightstrip.scheduler.setLightEffect(
                (gamePiece == CUBE) ? () -> {
                    lightstrip.flashingRGB(194, 3, 252);
                } : () -> {
                    lightstrip.flashingRGB(252, 211, 3);
                }, 14, 15, .1, "intake");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // runs until current spikes
        // NOTE: Change this to use time of flight sensor
        int statorThreshold = (gamePiece == CUBE) ? 30 : 70;

        double intakeSpeed = (gamePiece == CUBE) ? -.7 : .7;
        if (clawSubsystem.intake.getStatorCurrent() < statorThreshold) {
            clawSubsystem.setIntake(intakeSpeed);
        } else {
            // finish();
        }
        // clawSubsystem.setIntake(intakeSpeed);
        if (gamePiece.equals(CONE) &&
                !(RobotContainer.slideController.getAButton() && RobotContainer.slideController.getXButton())) {
            finish();
        } else if (gamePiece.equals(CUBE) &&
                !(RobotContainer.slideController.getBButton() && RobotContainer.slideController.getYButton())) {
            finish();
        }

    }

    public void finish() {
        robotContainer.setCurrentElement(gamePiece);
        robotContainer.setTargetElement(GamePiece.NONE);
        robotContainer.intakeFinish = true;
        if (lightstrip.scheduler.getCurrentEffectName().equals("intake")) {
            lightstrip.scheduler.setTimer(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        finish();
        // TODO:I have a feeling the currentelement might need to be NONE if interrupted
        robotContainer.intakeFinish = false;
        clawSubsystem.setIntake(0);
        robotContainer.setPositionLevel(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return robotContainer.getTargetElement() == NONE;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.RobotContainer.GamePiece.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Lights;

public class ClawIntake extends CommandBase {
    // this is a copy of ClawConeIntake, with minor adjustments. It should be
    // refactored to a single command with a piece type parameter.
    private GamePiece gamePiece;
    public IntakeSubsystem clawSubsystem;
    public RobotContainer robotContainer;
    public Lights lightstrip;

    private boolean autoOverride;
    private double delay;
    // private double timeout;

    private boolean currentSpikeDetected = false;

    private double startTime;

    /**
     * Creates a ClawIntake command.
     * 
     * @param gamePiece      game piece to intake.
     * @param clawSubsystem  intake subsystem.
     * @param robotContainer robot container.
     * @param autoOverride   whether the command is running in auto.
     * @param delay          delay to begin running the intake.
     * @param timeout        amount of time after the delay to run the robot for.
     */
    public ClawIntake(GamePiece gamePiece, IntakeSubsystem clawSubsystem, RobotContainer robotContainer,
            boolean autoOverride, double delay, double timeout) {
        this.gamePiece = gamePiece;
        this.clawSubsystem = clawSubsystem;
        this.robotContainer = robotContainer;
        this.autoOverride = autoOverride;
        this.delay = delay;
        // this.timeout = timeout;

        currentSpikeDetected = false;

        addRequirements(clawSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();

        currentSpikeDetected = false;

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
        int statorThreshold = (gamePiece == CUBE) ? 45 : 80;
        double intakeSpeed = (gamePiece == CUBE) ? -.7 : .7;
        double runningTime = Timer.getFPGATimestamp() - startTime;

        if (runningTime > delay && autoOverride) {
            if (gamePiece == CUBE) {
                intakeSpeed = -0.7;
            } else if (gamePiece == CONE) {
                intakeSpeed = 0.7;
            } else {
                intakeSpeed = 0;
            }
            // clawSubsystem.setIntake(intakeSpeed);
        }

        if (clawSubsystem.intake.getStatorCurrent() < statorThreshold) {
            if (currentSpikeDetected && autoOverride) {
                intakeSpeed = 0;
            } else {
                clawSubsystem.setIntake(intakeSpeed);
            }
        } else {
            intakeSpeed = 0;
            currentSpikeDetected = true;
        }

        clawSubsystem.setIntake(intakeSpeed);

        // clawSubsystem.setIntake(intakeSpeed);
        if (!autoOverride) {
            if (gamePiece.equals(CONE) &&
                    !(RobotContainer.slideController.getAButton() && RobotContainer.slideController.getXButton())) {
                finish();
            } else if (gamePiece.equals(CUBE) &&
                    !(RobotContainer.slideController.getBButton() && RobotContainer.slideController.getYButton())) {
                finish();
            }
        }

        if (currentSpikeDetected) {
            if (lightstrip.scheduler.getCurrentEffectName() != "current") {
                lightstrip.scheduler.setLightEffect(() -> {
                    lightstrip.setSolidRGB(255, 255, 255);
                }, .9, 25, .10, "current");
            } else {
                lightstrip.scheduler.setTimer(1);
            }
        }
        SmartDashboard.putBoolean("Current Spike Detected", currentSpikeDetected);
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

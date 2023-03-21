// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SlideConstants;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SlideCommand extends CommandBase {
    private SlideSubsystem slide;
    @SuppressWarnings("unused")
    private WristSubsystem wrist;
    public int bumperPos = 0;
    RobotContainer robotContainer;

    private boolean hasSpool, hasZeroed;

    // these will be the heights of the slide at different points. The height will
    // be set as SlideConstants.slideSetpoints[bumperPos]

    /** Creates a new SlideCommand. */
    public SlideCommand(SlideSubsystem slide, WristSubsystem wrist, RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // Use addRequirements() here to declare subsystem dependencies.
        this.slide = slide;
        this.wrist = wrist;
        wrist = robotContainer.wrist;

        hasSpool = !slide.getBottomLimitSwitch();
        hasZeroed = false;

        addRequirements(slide);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        slide.setSlide(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // slide.setSlide(robotContainer.firstController.getLeftY() / 3);

        if (bumperPos > 0) {
            robotContainer.speedMultiplier = .3;
        } else {
            robotContainer.speedMultiplier = .7;
        }

        double position = 0;

        if (!hasZeroed) {
            if (!hasSpool && slide.getBottomLimitSwitch()) {
                slide.setSlide(0.1);
            } else if (!hasSpool && !slide.getBottomLimitSwitch()) {
                hasSpool = true;
            } else if (hasSpool && !slide.getBottomLimitSwitch()) {
                slide.setSlide(-0.1);
            } else if (hasSpool && slide.getBottomLimitSwitch()) {
                hasZeroed = true;
            }

        } else {
            if (robotContainer.getPositionLevel() == 0) {
                position = 0;
                slide.setSlidePIDInches(position);

                if (slide.controller.atSetpoint() && !slide.getBottomLimitSwitch()) {
                    hasZeroed = false;
                }
            } else {
                if (robotContainer.getTargetElement().equals(GamePiece.CONE)) {
                    position = SlideConstants.coneIntakeSetpoints[robotContainer.getPositionLevel() - 1];

                } else if (robotContainer.getTargetElement().equals(GamePiece.CUBE)) {
                    position = SlideConstants.cubeIntakeSetpoints[robotContainer.getPositionLevel() - 1];

                } else if (robotContainer.getCurrentElement().equals(GamePiece.CONE)) {
                    position = SlideConstants.coneOuttakeSetpoint[robotContainer.getPositionLevel() - 1];

                } else if (robotContainer.getCurrentElement().equals(GamePiece.CUBE)) {
                    position = SlideConstants.cubeOuttakeSetpoint[robotContainer.getPositionLevel() - 1];
                }
                slide.setSlidePIDInches(position);

            }
        }

        if (slide.getTopLimitSwitch() && hasZeroed) {
            hasZeroed = false;
            hasSpool = false;
        }

        SmartDashboard.putBoolean("Has Zeroed", hasZeroed);
        SmartDashboard.putBoolean("Has Spool", hasSpool);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // make or call a function to set slide to zero sets slide back to zero
        bumperPos = 0;
        slide.setSlide(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

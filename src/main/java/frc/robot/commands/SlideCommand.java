// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SlideConstants;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.SlideSubsystem;

public class SlideCommand extends CommandBase {
    private SlideSubsystem slide;
    public int bumperPos = 0;
    RobotContainer robotContainer;
    private boolean hasSpool, hasZeroed;

    /** Creates a new SlideCommand. */
    public SlideCommand(SlideSubsystem slide, RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // Use addRequirements() here to declare subsystem dependencies.
        this.slide = slide;

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

        robotContainer.speedMultiplier = (bumperPos > 0) ? .3 : .7;

        double position = 0;

        boolean bottomLimitSwitch = slide.getBottomLimitSwitch();
        if (!hasZeroed) {
            if (!hasSpool && bottomLimitSwitch) {
                slide.setSlide(0.1);
            } else if (!hasSpool && !bottomLimitSwitch) {
                hasSpool = true;
            } else if (hasSpool && !bottomLimitSwitch) {
                slide.setSlide(-0.1);
            } else if (hasSpool && bottomLimitSwitch) {
                hasZeroed = true; // TODO: should we add a setSlide to zero here?
            }

        } else {
            int positionLevel = robotContainer.getPositionLevel();

            if (positionLevel == 0) {
                position = 0;
                slide.setSlidePIDInches(position);
                if (hasSpool && !slide.getBottomLimitSwitch()) {
                    slide.setSlide(-0.3);

                }

                // if (!slide.getBottomLimitSwitch()) {
                // hasZeroed = false;
                // }
            } else {
                if (robotContainer.getTargetElement().equals(GamePiece.CONE)) {
                    position = SlideConstants.coneIntakeSetpoints[positionLevel - 1];
                } else if (robotContainer.getTargetElement().equals(GamePiece.CUBE)) {
                    position = SlideConstants.cubeIntakeSetpoints[positionLevel - 1];
                } else if (robotContainer.getCurrentElement().equals(GamePiece.CONE)) {
                    position = SlideConstants.coneOuttakeSetpoint[positionLevel - 1];
                } else if (robotContainer.getCurrentElement().equals(GamePiece.CUBE)) {
                    position = SlideConstants.cubeOuttakeSetpoint[positionLevel - 1];
                }
                slide.setSlidePIDInches(position);
            }
        }

        // if (slide.getTopLimitSwitch() && hasZeroed) {
        // hasZeroed = false;
        // hasSpool = false;
        // }

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

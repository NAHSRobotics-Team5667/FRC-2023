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
    RobotContainer robotContainer;
    boolean autoOverride = false;
    boolean isCubeAuto = false;
    int positionAuto = 0;
    int positionLevel = 0;
    private boolean hasTension, hasZeroed;

    /** Creates a new SlideCommand. */
    public SlideCommand(SlideSubsystem slide, RobotContainer robotContainer, boolean autoOverride, int positionAuto,
            boolean isCubeAuto) {
        this.robotContainer = robotContainer;
        // Use addRequirements() here to declare subsystem dependencies.
        this.slide = slide;
        this.autoOverride = autoOverride;
        this.positionAuto = positionAuto;
        hasTension = false;
        hasZeroed = false;

        addRequirements(slide);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hasTension = false; // initialize boolean checkers just in case
        hasZeroed = false;
        slide.setSlide(0);
        if (autoOverride) {
            if (isCubeAuto) {
                robotContainer.setCurrentElement(GamePiece.CUBE);
            } else {
                robotContainer.setCurrentElement(GamePiece.CONE);
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Uncomment the following to control slide with stick
        // slide.setSlide(robotContainer.slideController.getLeftY() / 3);

        double position = 0; // initialize variable to hold position of slide
        boolean bottomLimitSwitch = slide.getBottomLimitSwitch();

        if (!hasZeroed) { // slide has not zeroed
            if (!hasTension) { // string may not have tension
                if (bottomLimitSwitch) { // bottom limit switch is hit
                    slide.setSlide(0.1); // go up slowly
                } else { // bottom limit switch is not hit
                    hasTension = true; // string has tension
                }
            } else { // string has tension
                if (!bottomLimitSwitch) { // bottom limit switch is not hit
                    slide.setSlide(-0.1); // go down slowly
                } else { // bottom limit switch is hit
                    hasZeroed = true; // slide has completed zeroing procedure
                    slide.setSlide(0); // stop slide in case motors have not updated
                }
            }
        } else {

            if (autoOverride) {
                this.positionLevel = positionAuto;

            } else {
                this.positionLevel = robotContainer.getPositionLevel();
            }

            if (positionLevel == 0) {
                position = -1; // set slide to go to -1 inches - eliminates any error
            } else { // position level is not equal to 0
                if (robotContainer.getTargetElement().equals(GamePiece.CONE)) {
                    position = SlideConstants.coneIntakeSetpoints[positionLevel - 1]; // length =
                    // 3

                } else if (robotContainer.getTargetElement().equals(GamePiece.CUBE)) {
                    position = SlideConstants.cubeIntakeSetpoints[positionLevel - 1]; // length =
                    // 1

                } else if (robotContainer.getCurrentElement().equals(GamePiece.CONE)) {
                    position = SlideConstants.coneOuttakeSetpoints[positionLevel - 1]; // length
                    // = 3

                } else if (robotContainer.getCurrentElement().equals(GamePiece.CUBE)) {
                    position = SlideConstants.cubeOuttakeSetpoints[positionLevel - 1]; // length
                    // = 3

                }
                // else { // current element is NONE and target element is NONE
                // position = -1; // reset position
                // }
            }
            slide.setSlidePIDInches(position); // set slide to go to position
        }

        if (RobotContainer.slideController.getLeftStickButtonPressed()) {
            hasZeroed = false;
            hasTension = false;
        }

        SmartDashboard.putBoolean("Has Zeroed", hasZeroed);
        SmartDashboard.putBoolean("Has Spool", hasTension);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        slide.setSlide(0); // stop slide
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

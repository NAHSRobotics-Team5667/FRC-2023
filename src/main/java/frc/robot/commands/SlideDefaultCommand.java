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

public class SlideDefaultCommand extends CommandBase {
    private SlideSubsystem slide;
    @SuppressWarnings("unused")
    private WristSubsystem wrist;
    public int bumperPos = 0;
    RobotContainer m_RobotContainer;

    private boolean hasSpool, hasZeroed;

    // these will be the heights of the slide at different points. The height will
    // be set as SlideConstants.slideSetpoints[bumperPos]

    /** Creates a new SlideCommand. */
    public SlideDefaultCommand(SlideSubsystem slide, WristSubsystem wrist, RobotContainer m_RobotContainer) {
        this.m_RobotContainer = m_RobotContainer;
        // Use addRequirements() here to declare subsystem dependencies.
        this.slide = slide;
        this.wrist = wrist;
        wrist = m_RobotContainer.m_wrist;

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
        if (bumperPos > 0) {
            m_RobotContainer.speedMultiplier = .3;
        } else {
            m_RobotContainer.speedMultiplier = .7;
        }
        // max right slide = 277000

        // slide.setSlide(MathUtil.applyDeadband(m_RobotContainer.m_controller.getLeftY()
        // / 2, 0.1));

        double position = 0;

        if (!hasZeroed) {
            if (!hasSpool && slide.getBottomLimitSwitch()) {
                slide.setSlide(0.1);
            } else if (!hasSpool && !slide.getBottomLimitSwitch()) {
                hasSpool = true;
                // slide.setSlide(0.1);
            } else if (hasSpool && !slide.getBottomLimitSwitch()) {
                slide.setSlide(-0.1);
            } else if (hasSpool && slide.getBottomLimitSwitch()) {
                hasZeroed = true;
            }
        } else {
            // slide.setSlide(MathUtil.applyDeadband(m_RobotContainer.m_controller.getLeftY()
            // // TESTING
            // / 2, 0.2));

            if (wrist.getBumperPos() == 0) {
                position = 0;
                slide.setSlide(-.35);
                // if (MathUtil.clamp(
                // //
                // slide.controller.calculate(SlideConstants.rawUnitsToInches(slide.getRightRawEncoder()),
                // // position),
                // // -0.9, 0.9) < .05) {
                // // slide.setSlide(-.1);
                // // } else {
                // // slide.setSlidePIDInches(position);
                // // }

                // slide.setSlidePIDInches(position);
                // if ( slide.controller.atSetpoint() && !slide.getBottomLimitSwitch()) {
                // hasZeroed = false;
                // }
            } else {
                if (m_RobotContainer.getTargetElement().equals(GamePiece.CONE)) {
                    position = SlideConstants.coneIntakeSetpoints[wrist.getBumperPos() - 1];

                } else if (m_RobotContainer.getTargetElement().equals(GamePiece.CUBE)) {
                    position = SlideConstants.cubeIntakeSetpoints[wrist.getBumperPos() - 1];

                } else if (m_RobotContainer.getCurrentElement().equals(GamePiece.CONE)) {
                    position = SlideConstants.coneOuttakeSetpoint[wrist.getBumperPos() - 1];

                } else if (m_RobotContainer.getCurrentElement().equals(GamePiece.CUBE)) {
                    position = SlideConstants.cubeOuttakeSetpoint[wrist.getBumperPos() - 1];
                }
                slide.setSlidePIDInches(position);

            }
        }

        if (slide.getTopLimitSwitch() && hasZeroed) {
            hasZeroed = false;
            hasSpool = false;
        }

        if (slide.getTopLimitSwitch()) {
            wrist.setBumperPos(0);
        }

        SmartDashboard.putBoolean("Has Zeroed", hasZeroed);
        SmartDashboard.putBoolean("Has Spool", hasSpool);
        // SmartDashboard.putNumber("Slide Position", position);

        // slide.setSlidePIDInches(30);
        // slide.setSlidePIDEncoder(113000);

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

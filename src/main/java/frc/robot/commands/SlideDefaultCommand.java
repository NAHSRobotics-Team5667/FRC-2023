// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

<<<<<<< HEAD
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
=======
>>>>>>> 714163e41e0b1bb3dc967718383352367aa723db
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

    // these will be the heights of the slide at different points. The height will be set as SlideConstants.slideSetpoints[bumperPos]

    /** Creates a new SlideCommand. */
    public SlideDefaultCommand(SlideSubsystem slide, WristSubsystem wrist, RobotContainer m_RobotContainer) {
        this.m_RobotContainer = m_RobotContainer;
        // Use addRequirements() here to declare subsystem dependencies.
        this.slide = slide;
        this.wrist = wrist;
        wrist = m_RobotContainer.m_wrist;
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
        // max right slide = 277000

        slide.setSlide(MathUtil.clamp(m_RobotContainer.m_controller.getLeftY(), -0.5, 0.5));

        double position = 0;
        
        /*if (wrist.getBumperPos() == 0) {
            position = 0;
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
        }*/

       // slide.setSlidePIDInches(position);

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

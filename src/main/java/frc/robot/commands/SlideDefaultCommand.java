// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SlideDefaultCommand extends CommandBase {
    private SlideSubsystem slide;
    @SuppressWarnings("unused")
    private WristSubsystem wrist;
    public int bumperPos = 0;

    // these will be the heights of the slide at different points. The height will be set as SlideConstants.slideSetpoints[bumperPos]

    /** Creates a new SlideCommand. */
    public SlideDefaultCommand(SlideSubsystem slide, WristSubsystem wrist, RobotContainer m_RobotContainer) {
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
        //slide.moveSlide(Setpoints[wrist.bumperPos]);
        slide.setSlide(0);
    }
    
    public void joystickControl() {}

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

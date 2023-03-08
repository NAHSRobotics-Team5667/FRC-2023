// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SlideSubsystem;

public class SlideCommand extends CommandBase {
    private SlideSubsystem slide;
    public double bumperPos = 0;
    public static double[] Setpoints = { 
        0,
        0,
        0,
        0
        // these will be the heights of the slide at different points. The height will be set as Setpoints[bumperPos]
        
    };

    /** Creates a new SlideCommand. */
    public SlideCommand(SlideSubsystem slide) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.slide = slide;
        addRequirements(slide);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        slide.setSlide(0);
        slide.setTilt(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        
       

    }
    public void joystickControl() {
        double bumperPos = 0;
        if (RobotContainer.m_controller.getLeftBumperPressed()) {
            if (bumperPos == 3){
                bumperPos= 3;
            }
            else{
                bumperPos++;
            }
        }
        if (RobotContainer.m_controller.getRightBumperPressed()) {
            if (bumperPos == 0){
                bumperPos= 0;
            }
            else{
                bumperPos =- 1;
            }
        }
        if (RobotContainer.m_controller.getYButton()) {
            bumperPos = 0;
        }
        if (RobotContainer.m_controller.getXButton()){
            bumperPos = 3;
        }


        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // make or call a function to set slide to zero sets slide back to zero
        bumperPos = 0;
        slide.setSlide(0);
        slide.setTilt(0); 
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

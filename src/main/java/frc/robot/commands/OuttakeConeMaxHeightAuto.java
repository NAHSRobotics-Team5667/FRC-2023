// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SlideConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class OuttakeConeMaxHeightAuto extends CommandBase {
    RobotContainer robotContainer;
    WristSubsystem wrist;
    IntakeSubsystem intake;
    SlideSubsystem slide;
    double positionSlide = 0, positionWrist = 0, timer = 0;
    int height;

    /** Creates a new OuttakeConeMaxHeightAuto. */
    public OuttakeConeMaxHeightAuto(RobotContainer robotContainer, WristSubsystem wrist, IntakeSubsystem intake,
            SlideSubsystem slide, int height) {
        this.robotContainer = robotContainer;
        this.height = height;
        this.intake = intake;
        this.wrist = wrist;
        this.slide = slide;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(wrist, intake, slide);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotContainer.setCurrentElement(GamePiece.CONE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        positionSlide = SlideConstants.coneOuttakeSetpoints[height];
        positionWrist = WristConstants.coneOuttakeSetpoints[height];
        if (timer < 3) {
            wrist.setPosition(positionWrist);
            slide.setSlidePIDInches(positionSlide);
            timer += .02;
        } else if (timer < 5) {
            intake.setIntake(-.45);
            timer += .02;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

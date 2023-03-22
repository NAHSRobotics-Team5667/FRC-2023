// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ClawConeIntake extends CommandBase {
    // TODO: this is a copy of ClawCubeIntake, with minor adjustments. It should be refactored to a single command with a piece type parameter.
    public IntakeSubsystem clawSubsystem;

    public WristSubsystem wrist;
    public boolean isCube;

    public RobotContainer robotContainer;
    // these will be the heights of the slide at different points. The height will
    // be set as ClawConstants.ClawSetpoints[bumperPos]

    /** Creates a new SlideIntakeAndOuttakeCommand. */
    public ClawConeIntake(IntakeSubsystem clawSubsystem, WristSubsystem wrist, boolean isCube,
            RobotContainer robotContainer) {
        this.clawSubsystem = clawSubsystem;
        this.wrist = wrist;
        this.robotContainer = robotContainer;

        // addRequirements(clawSubsystem, wrist);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        robotContainer.setTargetElement(GamePiece.CONE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // runs until current spikes
        // TODO: Change to time of flight sensor
        if (clawSubsystem.intake.getStatorCurrent() < 35) {
            clawSubsystem.setIntake(.45);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        robotContainer.intakeFinish = false;
        clawSubsystem.setIntake(0);

        robotContainer.setPositionLevel(0);

        robotContainer.setCurrentElement(GamePiece.CONE);
        robotContainer.setTargetElement(GamePiece.NONE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

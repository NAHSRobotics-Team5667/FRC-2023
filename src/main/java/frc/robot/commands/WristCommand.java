// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
    private WristSubsystem wrist;
    private RobotContainer robotContainer;

    /** Creates a new WristCommand. */
    public WristCommand(WristSubsystem wrist, RobotContainer robotContainer) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.wrist = wrist;
        this.robotContainer = robotContainer;
        addRequirements(wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        wrist.setWrist(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Uncomment below if controlling wrist with controller
        // wrist.setWrist(RobotContainer.slideController.getRightX() / 3);
        double position = WristConstants.kWristSafePosition;
        // default position is stowaway

        GamePiece currentElement = robotContainer.getCurrentElement(),
                targetElement = robotContainer.getTargetElement();

        if (robotContainer.getPositionLevel() == 0) {
            if (currentElement.equals(GamePiece.CONE)) {
                position = WristConstants.kConeSafePosition; // go back to cone stowaway
            }
        } else { // position level > 0
            if (targetElement.equals(GamePiece.CONE)) {
                position = WristConstants.coneIntakeSetpoints[robotContainer.getPositionLevel() - 1];
                // length = 3
            } else if (targetElement.equals(GamePiece.CUBE)) {
                position = WristConstants.cubeIntakeSetpoints[robotContainer.getPositionLevel() - 1];
                // length = 1
            } else if (currentElement.equals(GamePiece.CONE)) {
                position = WristConstants.coneOuttakeSetpoints[robotContainer.getPositionLevel() - 1];
                // length = 3
            } else if (currentElement.equals(GamePiece.CUBE)) {
                position = WristConstants.cubeOuttakeSetpoints[robotContainer.getPositionLevel() - 1];
                // length = 3
            }
        }
        wrist.setPosition(position);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wrist.setWrist(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

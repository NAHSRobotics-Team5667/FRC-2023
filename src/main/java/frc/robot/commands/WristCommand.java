// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;
import static frc.robot.RobotContainer.GamePiece.*;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
    private WristSubsystem wrist;
    private RobotContainer robotContainer;
    private boolean autoOverride, isCubeAuto = false;
    private int positionAuto = 0, index = 0;
    private double delay = 0, clock = 0;

    /** Creates a new WristCommand. */
    public WristCommand(WristSubsystem wrist, RobotContainer robotContainer, boolean autoOverride, int positionAuto,
            boolean isCubeAuto, double delay) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.wrist = wrist;
        this.delay = delay;
        this.isCubeAuto = isCubeAuto;
        this.positionAuto = positionAuto;
        this.autoOverride = autoOverride;
        this.robotContainer = robotContainer;
        addRequirements(wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        wrist.setWrist(0);
        if (autoOverride) {
            robotContainer.setCurrentElement(isCubeAuto ? CUBE : CONE);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Uncomment below if controlling wrist with controller
        // wrist.setWrist(RobotContainer.slideController.getRightX() / 3);

        double position = WristConstants.kWristSafePosition;
        isCubeAuto = false;
        String output = "nah";
        GamePiece currentElement = robotContainer.getCurrentElement(),
                targetElement = robotContainer.getTargetElement();

        if (autoOverride) {
            if (delay > clock) {
                clock += .02;
                index = positionAuto;
            } else {
                index = 0;
            }
        } else {
            index = robotContainer.getPositionLevel();
        }

        if (index == 0) {
            if (currentElement.equals(CONE)) {
                position = WristConstants.kConeSafePosition; // go back to cone stowaway
            }
        } else if (robotContainer.getPositionLevel() != 0 || autoOverride) { //
            // position level > 0
            output = "ya did it sport";
            if (targetElement.equals(CONE)) {
                position = WristConstants.coneIntakeSetpoints[index - 1];
                // length = 3
            } else if (targetElement.equals(CUBE)) {
                position = WristConstants.cubeIntakeSetpoints[index - 1];
                // length = 1
            } else if (currentElement.equals(CONE)) {
                position = WristConstants.coneOuttakeSetpoints[index - 1];
                // length = 3
            } else if (currentElement.equals(CUBE)) {
                position = WristConstants.cubeOuttakeSetpoints[index - 1];
                // length = 3
            }
        }
        wrist.setPosition(position);
        SmartDashboard.putString("WOO", output);
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

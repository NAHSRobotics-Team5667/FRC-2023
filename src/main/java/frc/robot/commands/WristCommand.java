// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
    WristSubsystem wrist;
    XboxController m_Controller = RobotContainer.m_controller;
    RobotContainer robotContainer;
    // these will be the heights of the slide at different points. The wrist angle
    // will be set as Setpoints[bumperPos]

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

        double position = 0;
        String safe = "";

        if (wrist.getBumperPos() == 0) {
            if (robotContainer.getCurrentElement().equals(GamePiece.CONE)) {
                position = -90;
            } else {
                position = WristConstants.kWristSafePosition;
                safe = "safe";
            }
        } else {
            if (robotContainer.getTargetElement().equals(GamePiece.CONE)) {
                position = WristConstants.coneIntakeSetpoints[wrist.getBumperPos() - 1];

            } else if (robotContainer.getTargetElement().equals(GamePiece.CUBE)) {
                position = WristConstants.cubeIntakeSetpoints[wrist.getBumperPos() - 1];

            } else if (robotContainer.getCurrentElement().equals(GamePiece.CONE)) {
                position = WristConstants.kWristConeOuttakeSetpoint[wrist.getBumperPos() -
                        1];

            } else if (robotContainer.getCurrentElement().equals(GamePiece.CUBE)) {

                position = WristConstants.kWristCubeOuttakeSetpoint[wrist.getBumperPos() -
                        1];
            }
            safe = "not";
        }

        SmartDashboard.putString("Safe", safe);

        wrist.setPosition(position);

        // wrist.setWrist(MathUtil.applyDeadband(RobotContainer.m_controller.getRightX()
        // // TESTING
        // / 2, 0.2));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wrist.setWrist(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Controller.getAButtonPressed() || m_Controller.getYButton();
    }
}

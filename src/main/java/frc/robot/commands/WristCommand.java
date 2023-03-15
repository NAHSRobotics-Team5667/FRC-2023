// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
    WristSubsystem wrist;
    XboxController m_Controller = RobotContainer.m_controller;
    // these will be the heights of the slide at different points. The wrist angle
    // will be set as Setpoints[bumperPos]

    /** Creates a new WristCommand. */
    public WristCommand(WristSubsystem wrist) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.wrist = wrist;
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
        // wrist.maintainSafePosition();

        // if (wrist.getEncoder() <= 0.1) {
        //     wrist.setWrist(0.1);
        // } else if (wrist.getEncoder() >= (225 / 360)) {
        //     wrist.setWrist(-0.1);
        // } else {
        //     wrist.setWrist(0.1);
        // }

        wrist.setPosition(-125);

        // wrist.setWrist(-0.1);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wrist.setWrist(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_Controller.getAButtonPressed() || m_Controller.getYButton()) {
            return true;
        } else {
            return false;
        }
    }
}

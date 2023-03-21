// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class WristConeIntake extends CommandBase {
    boolean isPieceIntaken;
    IntakeSubsystem claw;
    SlideSubsystem slide;
    WristSubsystem wrist;
    boolean isCube;
    double setpoint;
    boolean isDone = false;
    RobotContainer robotContainer;

    /** Creates a new IntakeOuttakeProcessCube. */
    public WristConeIntake(WristSubsystem wrist, boolean isCube, IntakeSubsystem claw,
            RobotContainer robotContainer) {
        this.isPieceIntaken = claw.isPieceIntaken();
        this.wrist = wrist;
        this.isCube = isCube;
        this.robotContainer = robotContainer;
        robotContainer.lightstrip.scheduler.setLightEffect(() -> {
            robotContainer.lightstrip.flashingRGB(isCube ? 255 : 120, isCube ? 210 : 0, isCube ? 0 : 153);
        }, 1.5, 10, .14);
        addRequirements(wrist);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        wrist.coneIntakeAngled();
        if (Math.abs(wrist.pidError()) < .1) {
            isDone = true;
            robotContainer.intakeFinish = true;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        robotContainer.intakeFinish = false;
        wrist.wristMotorFirst.setVoltage(0);
        wrist.wristMotorSecond.setVoltage(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

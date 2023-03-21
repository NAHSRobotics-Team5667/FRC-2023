// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

/** The DriveTrainCommand class */
public class DrivetrainCommand extends CommandBase {
    public DrivetrainSubsystem swerve;
    private boolean slowmode = false;
    public double speedMultiplier = .9;
    RobotContainer robotContainer;
    public boolean FieldOriented = true;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    // haha changing slewratelimiters go brrrrr
    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(2),
            yspeedLimiter = new SlewRateLimiter(2),
            rotLimiter = new SlewRateLimiter(3);

    /** Creates a new DrivetrainCommand. */
    public DrivetrainCommand(DrivetrainSubsystem drive, RobotContainer robotContainer) {
        addRequirements(drive);
        this.robotContainer = robotContainer;
        this.swerve = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerve.resetGyro();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        joystickDrive();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.swerve.driveVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    /*
     * Calculates the appropriate speeds from controller inputs, and sends them to
     * the drive subsystem
     */
    private void joystickDrive() {
        if (RobotContainer.firstController.getPOV() == 180) {
            FieldOriented = !FieldOriented;

        }
        if (RobotContainer.secondController.getLeftStickButtonPressed()) {
            slowmode = !slowmode;
        }
        if (slowmode) {
            speedMultiplier = .7;
        } else {
            speedMultiplier = .9;
        }
        if (RobotContainer.secondController.getRightStickButton()) {
            this.swerve.resetGyro();
        }

        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.

        double xSpeed = xspeedLimiter
                .calculate(MathUtil
                        .applyDeadband(-RobotContainer.secondController.getLeftX() * robotContainer.speedMultiplier,
                                0.1))
                * DrivetrainSubsystem.kMaxSpeed;

        xSpeed = MathUtil.applyDeadband(-RobotContainer.secondController.getLeftX() * robotContainer.speedMultiplier,
                0.1)
                * DrivetrainSubsystem.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this3 because
        // we want a positive value when we pull to the left. Xbox controller
        // return positive values when you pull to the right by default.

        double ySpeed = yspeedLimiter
                .calculate(MathUtil
                        .applyDeadband(-RobotContainer.secondController.getLeftY() * robotContainer.speedMultiplier,
                                0.15))
                * DrivetrainSubsystem.kMaxSpeed;

        ySpeed = MathUtil.applyDeadband(-RobotContainer.secondController.getLeftY() * robotContainer.speedMultiplier,
                0.15)
                * DrivetrainSubsystem.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.

        double rot = rotLimiter
                .calculate(MathUtil.applyDeadband(RobotContainer.secondController.getRightX() * 0.4, 0.15))
                * DrivetrainSubsystem.kMaxAngularSpeed;

        rot = MathUtil.applyDeadband(RobotContainer.secondController.getRightX(), 0.15)
                * DrivetrainSubsystem.kMaxAngularSpeed;

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rot", rot);

        SmartDashboard.putNumber("Left Y", RobotContainer.secondController.getLeftY());
        SmartDashboard.putNumber("Left X", RobotContainer.secondController.getLeftX());
        SmartDashboard.putNumber("Right X", RobotContainer.secondController.getRightX());

        this.swerve.drive(xSpeed, ySpeed, rot, FieldOriented);
    }
}

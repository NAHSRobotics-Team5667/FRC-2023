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
    public DrivetrainSubsystem drive;
    public double speedMultiplier = .9;
    // TODO: put this and the one in RobotContainer in Constants. I would also put
    // the slowmode multiplier in Constants.
    // OK

    RobotContainer robotContainer;
    public boolean fieldOriented = true, slowmode = false; // TODO: maybe put these in DrivetrainSubsystem?
    // No, they are ok here, this should be edited in command as it affects how fast
    // it drives, and how it drives, its easier to translate them in the drive
    // method

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    // haha changing slewratelimiters go brrrrr
    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(2),
            yspeedLimiter = new SlewRateLimiter(2),
            rotLimiter = new SlewRateLimiter(3);

    /** Creates a new DrivetrainCommand. */
    public DrivetrainCommand(DrivetrainSubsystem drive, RobotContainer robotContainer) {
        addRequirements(drive);
        this.robotContainer = robotContainer;
        this.drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.resetGyro();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        joystickDrive();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.drive.driveVoltage(0);
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
            fieldOriented = !fieldOriented;
        }

        if (RobotContainer.secondController.getLeftStickButtonPressed()) {
            slowmode = !slowmode;
        }
        speedMultiplier = slowmode ? .5 : .5;

        if (RobotContainer.secondController.getRightStickButton()) {
            this.drive.resetGyro();
        }

        if (robotContainer.getPositionLevel() >= 2) {
            speedMultiplier = 0.2;
        } else {
            speedMultiplier = 0.5;
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

        this.drive.drive(xSpeed, ySpeed, rot, fieldOriented);
    }
}

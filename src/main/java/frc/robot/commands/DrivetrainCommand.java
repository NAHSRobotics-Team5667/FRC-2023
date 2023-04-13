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
import frc.robot.subsystems.SlideSubsystem;

/** The DriveTrainCommand class */
public class DrivetrainCommand extends CommandBase {
    private DrivetrainSubsystem drive;
    private RobotContainer robotContainer;
    private boolean fieldOriented = true;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    // haha changing slewratelimiters go brrrrr
    @SuppressWarnings("unused")
    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(2),
            yspeedLimiter = new SlewRateLimiter(2),
            rotLimiter = new SlewRateLimiter(3);

    /** Creates a new DrivetrainCommand. */
    public DrivetrainCommand(DrivetrainSubsystem drive, RobotContainer robotContainer) {
        fieldOriented = true; // set field oriented to true by default
        this.robotContainer = robotContainer;
        this.drive = drive;

        addRequirements(drive);
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
        if (RobotContainer.driveController.getRightBumperPressed()) {
            fieldOriented = !fieldOriented;
        }

        if (RobotContainer.driveController.getRightStickButton()) {
            this.drive.resetGyro();
        }

        robotContainer.setSpeedMultiplier((120 - robotContainer.getSlide().getSlideHeightInches()) / 120);
        robotContainer.setTurnMultiplier((120 - robotContainer.getSlide().getSlideHeightInches()) / 120);

        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.

        double leftX = (fieldOriented) ? -RobotContainer.driveController.getLeftX()
                : RobotContainer.driveController.getLeftX();

        double leftY = (fieldOriented) ? -RobotContainer.driveController.getLeftY()
                : RobotContainer.driveController.getLeftY();

        double xSpeed = MathUtil.applyDeadband(
                leftX * robotContainer.getSpeedMultiplier(),
                0.1)
                * DrivetrainSubsystem.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this3 because
        // we want a positive value when we pull to the left. Xbox controller
        // return positive values when you pull to the right by default.

        double ySpeed = MathUtil.applyDeadband(
                leftY * robotContainer.getSpeedMultiplier(),
                0.15)
                * DrivetrainSubsystem.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.

        double rot = MathUtil.applyDeadband(
                RobotContainer.driveController.getRightX() * robotContainer.getTurnMultiplier(), 0.15)
                * DrivetrainSubsystem.kMaxAngularSpeed;

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rot", rot);

        SmartDashboard.putNumber("Left Y", RobotContainer.driveController.getLeftY());
        SmartDashboard.putNumber("Left X", RobotContainer.driveController.getLeftX());
        SmartDashboard.putNumber("Right X", RobotContainer.driveController.getRightX());

        this.drive.drive(xSpeed, ySpeed, rot, fieldOriented);
    }
}

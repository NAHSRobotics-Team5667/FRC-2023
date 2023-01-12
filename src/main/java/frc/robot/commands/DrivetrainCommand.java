// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainCommand extends CommandBase {
	private DrivetrainSubsystem m_swerve;

	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

	/** Creates a new DrivetrainCommand. */
	public DrivetrainCommand(DrivetrainSubsystem drive) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drive);
		this.m_swerve = drive;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		joystickDrive();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_swerve.driveVoltage(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	private void joystickDrive() {
		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final double xSpeed = -m_xspeedLimiter
				.calculate(MathUtil.applyDeadband(RobotContainer.m_controller.getLeftY(), 0.02))
				* DrivetrainSubsystem.kMaxSpeed;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final double ySpeed = m_yspeedLimiter
				.calculate(MathUtil.applyDeadband(RobotContainer.m_controller.getLeftX(), 0.02))
				* DrivetrainSubsystem.kMaxSpeed;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		final double rot = -m_rotLimiter
				.calculate(MathUtil.applyDeadband(RobotContainer.m_controller.getRightX(), 0.02))
				* DrivetrainSubsystem.kMaxAngularSpeed;

		m_swerve.drive(xSpeed, ySpeed, rot, false);
	}
}

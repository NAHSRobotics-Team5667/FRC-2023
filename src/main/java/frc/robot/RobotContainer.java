// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.subsystems.DrivetrainAutoSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	// private DrivetrainSubsystem drive = new DrivetrainSubsystem();
	public static final XboxController m_controller = new XboxController(0);
	private DrivetrainAutoSubsystem m_auto;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// This will load the file "FullAuto.path" and generate it with a max velocity
		// of 4 m/s and a max acceleration of 3 m/s^2
		// for every path in the group
		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

		// This is just an example event map. It would be better to have a constant,
		// global event map
		// in your code that will be used by all path following commands.
		HashMap<String, Command> eventMap = new HashMap();
		eventMap.put("marker1", new PrintCommand("Passed marker 1"));

		Supplier<Pose2d> poseSupplier = new Supplier<Pose2d>() {
			@Override
			public Pose2d get() {
				return m_auto.getPositionPose2d();
			}
		};

		Consumer<Pose2d> resetPoseConsumer = new Consumer<Pose2d>() {
			@Override
			public void accept(Pose2d pose) {
				// TODO: MAKE THIS THE CORRECT MODULE POSITIONS
				SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {};
				m_auto.resetPose(pose.getRotation(), modulePositions, pose);
			}
		};

		Consumer<SwerveModuleState[]> outputModuleConsumer = new Consumer<SwerveModuleState[]>() {
			@Override
			public void accept(SwerveModuleState[] t) {
				m_auto.pleaseGodLetThisWork(t[0], t[1], t[2], t[3]);
			}
		};

		// Create the AutoBuilder. This only needs to be created once when robot code
		// starts, not every time you want to create an auto command. A good place to
		// put this is in RobotContainer along with your subsystems.
		SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
				poseSupplier, // Pose2d supplier
				resetPoseConsumer, // Pose2d consumer, used to reset odometry at the beginning of auto
				m_auto.m_kinematics, // SwerveDriveKinematics
				new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
													// and Y PID controllers)
				new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
													// rotation controller)
				outputModuleConsumer, // Module states consumer used to output to the drive subsystem
				eventMap,
				true, // Should the path be automatically mirrored depending on alliance color.
						// Optional, defaults to true
				m_auto // The drive subsystem. Used to properly set the requirements of path following
						// commands
		);

		Command fullAuto = autoBuilder.fullAuto(pathGroup);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return null;
	}
}

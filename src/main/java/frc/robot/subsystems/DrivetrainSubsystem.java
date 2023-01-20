// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
	public static final double kMaxSpeed = 5; // 3 meters per second
	public static final double kMaxAngularSpeed = 2* Math.PI; // 1/2 rotation per second

	private static final Translation2d m_frontLeftLocation = new Translation2d(0.417, -0.417);
	private final static Translation2d m_frontRightLocation = new Translation2d(0.417, 0.417);
	private final static Translation2d m_backLeftLocation = new Translation2d(-0.417, -0.417);
	private final static Translation2d m_backRightLocation = new Translation2d(-0.417, 0.417);

	public static SwerveModule m_frontLeft = new SwerveModule(15, 16, 0);
	public static SwerveModule m_frontRight = new SwerveModule(9, 10, 0);
	public static SwerveModule m_backLeft = new SwerveModule(14, 13, 0);
	public static SwerveModule m_backRight = new SwerveModule(12, 11, 0);

	private final AHRS m_gyro = new AHRS(Port.kMXP);

	public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), null);

	private SwerveModuleState[] m_swerveModuleStates = new SwerveModuleState[] {};
	private SwerveModulePosition[] m_swerveModuleFakeStates = new SwerveModulePosition[3];
	//private SwerveModulePosition m_swerveModulePositions = new SwerveModulePosition();
	public DrivetrainSubsystem() {
		m_gyro.reset();
	}
	
	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates = m_kinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		m_swerveModuleStates = swerveModuleStates;
		 
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);
	}
	public void fakeConverter(SwerveModuleState[] param){
		for (int i = 0; i < 4; i++) {
		param = m_swerveModuleStates;
		SwerveModulePosition[] output = m_swerveModuleFakeStates;
		m_swerveModuleStates[i].speedMetersPerSecond  = m_swerveModuleFakeStates[i].distanceMeters;
		m_swerveModuleFakeStates[i].angle = m_swerveModuleFakeStates[i].angle;

		}

	}
	public void driveVoltage(double voltage) {
		m_frontLeft.driveVoltage(voltage);
		m_frontRight.driveVoltage(voltage);
		m_backLeft.driveVoltage(voltage);
		m_backRight.driveVoltage(voltage);
	}

	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		m_odometry.update(
				m_gyro.getRotation2d(),
				m_swerveModuleFakeStates);
	}

	@Override
	public void periodic() {
		fakeConverter(m_swerveModuleStates);
		//public int checkBumper
		SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
		//SmartDashboard.putNumber("Abs Encoder", m_backRight.getAbsoluteEncoder());
		SmartDashboard.putNumber("FRA-Actual", m_frontRight.getTurnEncoderDistance());
		SmartDashboard.putNumber("FLA-Actual", m_frontLeft.getTurnEncoderDistance());
		SmartDashboard.putNumber("BRA-Actual", m_backRight.getTurnEncoderDistance());
		SmartDashboard.putNumber("BLA-Actual", m_backLeft.getTurnEncoderDistance());

		
		SmartDashboard.putNumber("FRA-Setpoint", m_frontRight.getAngleSetpoint());
		SmartDashboard.putNumber("FLA-Setpoint", m_frontLeft.getAngleSetpoint());
		SmartDashboard.putNumber("BRA-Setpoint", m_backRight.getAngleSetpoint());
		SmartDashboard.putNumber("BLA-Setpoint", m_backLeft.getAngleSetpoint());
		

		SmartDashboard.putNumber("FRD-Actual", m_frontRight.getDriveEncoderDistance());
		SmartDashboard.putNumber("FLD-Actual", m_frontLeft.getDriveEncoderDistance());
		SmartDashboard.putNumber("BRD-Actual", m_backRight.getDriveEncoderDistance());
		SmartDashboard.putNumber("BLD-Actual", m_backLeft.getDriveEncoderDistance());

		SmartDashboard.putNumber("FRD-Setpoint", m_frontRight.getDriveSetpoint());
		SmartDashboard.putNumber("FLD-Setpoint", m_frontLeft.getDriveSetpoint());
		SmartDashboard.putNumber("BRD-Setpoint", m_backRight.getDriveSetpoint());
		SmartDashboard.putNumber("BLD-Setpoint", m_backLeft.getDriveSetpoint());
		
	}
}
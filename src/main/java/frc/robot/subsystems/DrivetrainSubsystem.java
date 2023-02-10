// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Timer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
	public static final double kMaxSpeed = DriveConstants.kMaxSpeed; // 5 is 3 meters per second
	public static final double kMaxAngularSpeed = DriveConstants.kMaxAngularSpeed; // 2 is 1/2 rotation per second
	public static DutyCycleEncoder FREncoder = new DutyCycleEncoder(DriveConstants.FREncoderID);
	public static DutyCycleEncoder FLEncoder = new DutyCycleEncoder(DriveConstants.FLEncoderID);
	public static DutyCycleEncoder BREncoder = new DutyCycleEncoder(DriveConstants.BREncoderID);
	public static DutyCycleEncoder BLEncoder = new DutyCycleEncoder(DriveConstants.BLEncoderID);

	public static SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveID, DriveConstants.kFrontLeftTurningID, DriveConstants.FLEncoderOffset, FLEncoder);
	public static SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveID, DriveConstants.kFrontRightTurningID, DriveConstants.FREncoderOffset, FREncoder);
	public static SwerveModule m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveID, DriveConstants.kBackLeftTurningID, DriveConstants.BLEncoderOffset, BLEncoder);
	public static SwerveModule m_backRight = new SwerveModule(DriveConstants.kBackRightDriveID, DriveConstants.kBackRightTurningID, DriveConstants.BREncoderOffset, BREncoder);
	

	private final SwerveModulePosition[] positions = {
		DrivetrainSubsystem.m_frontLeft.getPosition(), 
		DrivetrainSubsystem.m_frontRight.getPosition(), 
		DrivetrainSubsystem.m_backLeft.getPosition(), 
		DrivetrainSubsystem.m_backRight.getPosition()
	};
	
	private static final Translation2d m_frontLeftLocation = new Translation2d(0.417, -0.417);
	private final static Translation2d m_frontRightLocation = new Translation2d(0.417, 0.417);
	private final static Translation2d m_backLeftLocation = new Translation2d(-0.417, -0.417);
	private final static Translation2d m_backRightLocation = new Translation2d(-0.417, 0.417);
	
	public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
		m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	public final AHRS m_gyro = new AHRS(Port.kMXP);
		
	public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), positions);

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
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);

	}
	/*public void fakeConverter(SwerveModuleState[] param){
		for (int i = 0; i < 4; i++) {
		param = m_swerveModuleStates;
		SwerveModulePosition[] output = m_swerveModuleFakeStates;
		m_swerveModuleStates[i].speedMetersPerSecond  = m_swerveModuleFakeStates[i].distanceMeters;
		m_swerveModuleFakeStates[i].angle = m_swerveModuleFakeStates[i].angle;

		}

	}*/
	public void resetPose(Rotation2d gyro, SwerveModulePosition[] positions, Pose2d m_pose){
        // m_odometry.resetPosition(null, m_swerveModuleFakeStates, m_pose);
        m_odometry.resetPosition(gyro, positions, m_pose);
    }
	public void pleaseGodLetThisWork(SwerveModuleState Wheel1, SwerveModuleState Wheel2, SwerveModuleState Wheel3, SwerveModuleState Wheel4){
        m_frontLeft.setDesiredState(Wheel1);
		m_frontRight.setDesiredState(Wheel2);
		m_backLeft.setDesiredState(Wheel3);
		m_backRight.setDesiredState(Wheel4);
    }
	public Pose2d getPositionPose2d(){
        return m_odometry.getPoseMeters();
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
				positions);
	}
	

	@Override
	public void periodic() {
		//fakeConverter(m_swerveModuleStates);
		//public int checkBumper
		
		SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
		SmartDashboard.putNumber("absolute encoder heckin value", m_frontRight.trueEncoderOffset);
		//SmartDashboard.putNumber("Abs Encoder", m_backRight.getAbsoluteEncoder());
		SmartDashboard.putString("State angle", m_frontRight.getState().angle.toString());
		SmartDashboard.putNumber("FRATICK", m_frontRight.getTurnEncoderDistance());
		SmartDashboard.putNumber("FLATICK", m_frontLeft.getTurnEncoderDistance());
		SmartDashboard.putNumber("BRATICK", m_backRight.getTurnEncoderDistance());
		SmartDashboard.putNumber("FRATICK", m_backLeft.getTurnEncoderDistance());
		SmartDashboard.putNumber("FRA-ActualPS", m_frontRight.getTurnEncoderDistance());
		SmartDashboard.putNumber("FLA-ActualPS", m_frontLeft.getTurnEncoderDistance());
		SmartDashboard.putNumber("BRA-ActualPS", m_backRight.getTurnEncoderDistance());
		SmartDashboard.putNumber("BLA-ActualPS", m_backLeft.getTurnEncoderDistance());
		SmartDashboard.putNumber("FRA-Actual", (FREncoder.getAbsolutePosition()-m_frontRight.angleOffset)*2*Math.PI);
		SmartDashboard.putNumber("FLA-Actual", (FLEncoder.getAbsolutePosition()-m_frontLeft.angleOffset)*2*Math.PI);
		SmartDashboard.putNumber("BRA-Actual", (BREncoder.getAbsolutePosition()-m_backRight.angleOffset)*2*Math.PI);
		SmartDashboard.putNumber("BLA-Actual", (BLEncoder.getAbsolutePosition()-m_backLeft.angleOffset)*2*Math.PI);
		
		SmartDashboard.putNumber("FRA-Setpoint", m_frontRight.getAngleSetpoint());
		SmartDashboard.putNumber("FLA-Setpoint", m_frontLeft.getAngleSetpoint());
		SmartDashboard.putNumber("BRA-Setpoint", m_backRight.getAngleSetpoint());
		SmartDashboard.putNumber("BLA-Setpoint", m_backLeft.getAngleSetpoint());
		
/* 		SmartDashboard.putNumber("FRD-Actual", m_frontRight.getDriveEncoderDistance());
		SmartDashboard.putNumber("FLD-Actual", m_frontLeft.getDriveEncoderDistance());
		SmartDashboard.putNumber("BRD-Actual", m_backRight.getDriveEncoderDistance());
		SmartDashboard.putNumber("BLD-Actual", m_backLeft.getDriveEncoderDistance());

		SmartDashboard.putNumber("FRD-Setpoint", m_frontRight.getDriveSetpoint());
		SmartDashboard.putNumber("FLD-Setpoint", m_frontLeft.getDriveSetpoint());
		SmartDashboard.putNumber("BRD-Setpoint", m_backRight.getDriveSetpoint());
		SmartDashboard.putNumber("BLD-Setpoint", m_backLeft.getDriveSetpoint());*/
		
	}
}
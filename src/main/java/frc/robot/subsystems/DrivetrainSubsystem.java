// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
	public static final double kMaxSpeed = DriveConstants.kMaxSpeed; // 5 is 3 meters per second
	public static final double kMaxAngularSpeed = DriveConstants.kMaxAngularSpeed; // 2 is 1/2 rotation per second
	public static DutyCycleEncoder FREncoder = new DutyCycleEncoder(DriveConstants.FREncoderID);
	public static DutyCycleEncoder FLEncoder = new DutyCycleEncoder(DriveConstants.FLEncoderID);
	public static DutyCycleEncoder BREncoder = new DutyCycleEncoder(DriveConstants.BREncoderID);
	public static DutyCycleEncoder BLEncoder = new DutyCycleEncoder(DriveConstants.BLEncoderID);
	public Pose2d m_pose = new Pose2d();
	public static SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveID, DriveConstants.kFrontLeftTurningID, DriveConstants.FLEncoderOffset, FLEncoder);
	public static SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveID, DriveConstants.kFrontRightTurningID, DriveConstants.FREncoderOffset, FREncoder);
	public static SwerveModule m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveID, DriveConstants.kBackLeftTurningID, DriveConstants.BLEncoderOffset, BLEncoder);
	public static SwerveModule m_backRight = new SwerveModule(DriveConstants.kBackRightDriveID, DriveConstants.kBackRightTurningID, DriveConstants.BREncoderOffset, BREncoder);

	Vision vision;
	DifferentialDrivePoseEstimator poseEstimator;

	//loggables
	public boolean isHighGear;
	Field2d field;
	double distanceToSubstation = -1;
	DoubleArrayPublisher posePub;

	 //Drive Control
  	DifferentialDriveWheelVoltages wheelVolts;
  	DifferentialDriveWheelSpeeds currentDesiredWheelSpeeds;
	LTVDifferentialDriveController ltv;

	Translation2d Station = new Translation2d(0, 0);
  	Timer trajectoryTimer = new Timer();
	


	public static Rotation2d gyroOffset;

	public static SwerveModulePosition[] positions = {
		DrivetrainSubsystem.m_frontLeft.getPosition(), 
		DrivetrainSubsystem.m_frontRight.getPosition(), 
		DrivetrainSubsystem.m_backLeft.getPosition(), 
		DrivetrainSubsystem.m_backRight.getPosition()
	};
		
	//private Translation2d m_center = new Translation2d(0, 0);
	private Translation2d m_frontLeftLocation = new Translation2d(0.417, -0.417);
	private Translation2d m_frontRightLocation = new Translation2d(0.417, 0.417);
	private Translation2d m_backLeftLocation = new Translation2d(-0.417, -0.417);
	private Translation2d m_backRightLocation = new Translation2d(-0.417, 0.417);
	
	public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
		m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	public final AHRS m_gyro = new AHRS(Port.kMXP);
		
	public final SwerveDriveOdometry m_odometry;

	public DrivetrainSubsystem() {
		this.resetGyro();
	
		this.m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), positions, m_pose);
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
	
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Vision vision) {
		var swerveModuleStates = m_kinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.getGyro())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);

		posePub = NetworkTableInstance.getDefault().getTable("Poses").getDoubleArrayTopic("RobotPose").publish();
		
		field = new Field2d();
	
		poseEstimator = new DifferentialDrivePoseEstimator(
		  DriveConstants.kinematics,
		  new Rotation2d(),
		  m_frontLeft.getDriveEncoderDistance(),
		  m_frontRight.getDriveEncoderDistance(),
		  new Pose2d(1, 1, Rotation2d.fromDegrees(0))
		);
	}



  private double getAngularVelocityRadsPerSecond(double linearVel, double curvature) {
    return linearVel * curvature;
  }
  public Command followTrajectory(Trajectory trajectory) {
    PathPlannerServer.sendActivePath(trajectory.getStates());
    return runOnce(
      () -> {
        poseEstimator.resetPosition(Rotation2d.fromDegrees(-m_gyro.getAngle()), m_frontLeft.getDriveEncoderDistance(), m_frontRight.getDriveEncoderDistance(),trajectory.getInitialPose());
        trajectoryTimer.start();
      }).andThen(
        run(() -> {
          field.getObject("traj").setTrajectory(trajectory);
          Trajectory.State trajState = trajectory.sample(trajectoryTimer.get());
          PathPlannerServer.sendPathFollowingData(trajState.poseMeters, poseEstimator.getEstimatedPosition());
          currentDesiredWheelSpeeds = DriveConstants.kinematics.toWheelSpeeds(new ChassisSpeeds(trajState.velocityMetersPerSecond, 0, getAngularVelocityRadsPerSecond(trajState.velocityMetersPerSecond, trajState.curvatureRadPerMeter)));
          var nextState = trajectory.sample(trajectoryTimer.get()+0.02);
          var nextWheelSpeeds = DriveConstants.kinematics.toWheelSpeeds(new ChassisSpeeds(nextState.velocityMetersPerSecond, 0, getAngularVelocityRadsPerSecond(nextState.velocityMetersPerSecond, nextState.curvatureRadPerMeter)));
          var ltvVolts = ltv.calculate(
            poseEstimator.getEstimatedPosition(),
            currentDesiredWheelSpeeds.leftMetersPerSecond,
            currentDesiredWheelSpeeds.rightMetersPerSecond,
            trajState
          );
		  
        })
        .until(() -> ltv.atReference())
        .finallyDo((boolean end ) -> {
          trajectoryTimer.stop();
          trajectoryTimer.reset();
        }
      )
    );
  }


  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }
	
	public void resetPose(SwerveModulePosition[] positions, Pose2d m_pose){
        // m_odometry.resetPosition(null, m_swerveModuleFakeStates, m_pose);
        m_odometry.resetPosition(this.getGyro(), positions, m_pose);
    }
	public void pleaseGodLetThisWork(SwerveModuleState Wheel1, SwerveModuleState Wheel2, SwerveModuleState Wheel3, SwerveModuleState Wheel4){
        m_frontLeft.setDesiredState(Wheel1);
		m_frontRight.setDesiredState(Wheel2);
		m_backLeft.setDesiredState(Wheel3);
		m_backRight.setDesiredState(Wheel4);
    }
	public Pose2d getPositionPose2d(){
		
		//m_center.getX() + current
        return m_odometry.getPoseMeters();
    }
	public void driveVoltage(double voltage) {
		m_frontLeft.driveVoltage(voltage);
		m_frontRight.driveVoltage(voltage);
		m_backLeft.driveVoltage(voltage);
		m_backRight.driveVoltage(voltage);
	}

	/** Updates the field relative position of the robot. */
	
	public Rotation2d getGyro() {
		return(this.m_gyro.getRotation2d().minus(gyroOffset));
	}

	public void resetGyro() {
		gyroOffset = m_gyro.getRotation2d();

	}

	@Override
	public void periodic() {
		//fakeConverter(m_swerveModuleStates);
		//public int checkBumper
		Rotation2d gyroAngle = getGyro();
		m_pose = m_odometry.update(gyroAngle, new SwerveModulePosition[]{
			DrivetrainSubsystem.m_frontLeft.getPosition(), 
			DrivetrainSubsystem.m_frontRight.getPosition(), 
			DrivetrainSubsystem.m_backLeft.getPosition(), 
			DrivetrainSubsystem.m_backRight.getPosition()
		});
		SmartDashboard.putString("Gyro", m_frontLeftLocation.toString());
		SmartDashboard.putString("GyroFake", this.getGyro().toString());
		//SmartDashboard.putNumber("absolute encoder heckin value", m_frontRight.trueEncoderOffset);
		SmartDashboard.putString("hecking auto", this.m_pose.toString());
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
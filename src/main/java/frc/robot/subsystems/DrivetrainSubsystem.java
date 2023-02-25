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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = DriveConstants.kMaxSpeed; // 5 is 3 meters per second
    public static final double kMaxAngularSpeed = DriveConstants.kMaxAngularSpeed; // 2 is 1/2 rotation per second

    public static DutyCycleEncoder // Create the encoders
        FREncoder = new DutyCycleEncoder(DriveConstants.FREncoderID),
        FLEncoder = new DutyCycleEncoder(DriveConstants.FLEncoderID),
        BREncoder = new DutyCycleEncoder(DriveConstants.BREncoderID),
        BLEncoder = new DutyCycleEncoder(DriveConstants.BLEncoderID);

    public static SwerveModule // Create the Swerve Modules
        m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveID, DriveConstants.kFrontLeftTurningID,DriveConstants.FLEncoderOffset, FLEncoder),
        m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveID, DriveConstants.kFrontRightTurningID,DriveConstants.FREncoderOffset, FREncoder),
        m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveID, DriveConstants.kBackLeftTurningID,DriveConstants.BLEncoderOffset, BLEncoder),
        m_backRight = new SwerveModule(DriveConstants.kBackRightDriveID, DriveConstants.kBackRightTurningID,DriveConstants.BREncoderOffset, BREncoder);

    public static SwerveModulePosition[] positions = {
        DrivetrainSubsystem.m_frontLeft.getPosition(),
        DrivetrainSubsystem.m_frontRight.getPosition(),
        DrivetrainSubsystem.m_backLeft.getPosition(),
        DrivetrainSubsystem.m_backRight.getPosition()
    };
    private Translation2d // Create the locations of the wheels
        m_frontLeftLocation = new Translation2d(0.257, -0.306), //TODO: Get actual wheel positions
        //0.306
        //0.257
        m_frontRightLocation = new Translation2d(0.257, 0.306),
        m_backLeftLocation = new Translation2d(-0.257, -0.306),
        m_backRightLocation = new Translation2d(-0.257, 0.306);

    public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    public final AHRS m_gyro = new AHRS(Port.kMXP);
    public Rotation2d gyroOffset = m_gyro.getRotation2d();
    public Pose2d m_pose = new Pose2d(0, 0, getGyro());
    public final SwerveDriveOdometry m_odometry;

    // *Constructor for DrivetrainSubsystem - I 'ardly know 'er! */
    //dammit liam i didnt laugh
    public DrivetrainSubsystem() {
        m_gyro.calibrate(); // this probably isn't necessary but idk
        this.resetGyro(); // this however, is necessary ish

        this.m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), positions, m_pose);
        m_odometry.resetPosition(this.getGyro(), positions, m_pose);
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
        if (xSpeed != 0 || ySpeed != 0 || rot != 0) {
            var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.getGyro())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
            m_frontLeft.setDesiredState(swerveModuleStates[0]);
            m_frontRight.setDesiredState(swerveModuleStates[1]);
            m_backLeft.setDesiredState(swerveModuleStates[2]);
            m_backRight.setDesiredState(swerveModuleStates[3]);
        }
        else{
            m_frontLeft.driveVoltage(0);
            m_backRight.driveVoltage(0);
            m_backLeft.driveVoltage(0);
            m_frontRight.driveVoltage(0);
            
            
        }
      
    }

    /*
    public void fakeConverter(SwerveModuleState[] param){
        for (int i = 0; i < 4; i++) {
            param = m_swerveModuleStates;
            SwerveModulePosition[] output = m_swerveModuleFakeStates;
            m_swerveModuleStates[i].speedMetersPerSecond =
            m_swerveModuleFakeStates[i].distanceMeters;
            m_swerveModuleFakeStates[i].angle = m_swerveModuleFakeStates[i].angle;
        }
    }
     */
    public void resetPose(SwerveModulePosition[] positions, Pose2d m_pose) {
        // m_odometry.resetPosition(null, m_swerveModuleFakeStates, m_pose);
        m_odometry.resetPosition(this.getGyro(), positions, m_pose);
    }

    public void pleaseGodLetThisWork(SwerveModuleState Wheel1, SwerveModuleState Wheel2, SwerveModuleState Wheel3, SwerveModuleState Wheel4) {
        m_frontLeft.setDesiredState(Wheel1);
        m_frontRight.setDesiredState(Wheel2);
        m_backLeft.setDesiredState(Wheel3);
        m_backRight.setDesiredState(Wheel4);
    }

    public Pose2d getPositionPose2d() {

        // m_center.getX() + current
        return m_odometry.getPoseMeters();
    }

    public void driveVoltage(double voltage) {
        m_frontLeft.driveVoltage(voltage);
        m_frontRight.driveVoltage(voltage);
        m_backLeft.driveVoltage(voltage);
        m_backRight.driveVoltage(voltage);
    }

    /** Returns the rotation relative to the last resetGyro() called */
    public Rotation2d getGyro() {
        return (this.m_gyro.getRotation2d().minus(gyroOffset));
    }

    /**
     * Sets the offset used in
     * {@link frc.robot.subsystems.DrivetrainSubsystem#getGyro()}
     */
    public void resetGyro() {
        gyroOffset = m_gyro.getRotation2d();
    }

    @Override
    public void periodic() {
        Rotation2d gyroAngle = getGyro();
        m_pose = m_odometry.update(gyroAngle, new SwerveModulePosition[] {
            DrivetrainSubsystem.m_frontLeft.getPosition(),
            DrivetrainSubsystem.m_frontRight.getPosition(),
            DrivetrainSubsystem.m_backLeft.getPosition(),
            DrivetrainSubsystem.m_backRight.getPosition()
        });
      //  SmartDashboard.putString("Gyro", .getGyro().toString());
        SmartDashboard.putString("GyroFake", this.getGyro().toString());
        SmartDashboard.putNumber("FR Pose", m_frontRight.getTurnEncoderDistance());
        SmartDashboard.putNumber("BR Pose", m_backRight.getTurnEncoderDistance());
        SmartDashboard.putNumber("FL Pose", m_frontLeft.getTurnEncoderDistance());
        SmartDashboard.putNumber("BL Pose", m_backLeft.getTurnEncoderDistance());
        SmartDashboard.putNumber("FRA ABS", m_frontRight.Encoder.getAbsolutePosition());
        SmartDashboard.putNumber("FLA ABS", m_frontLeft.Encoder.getAbsolutePosition());
        SmartDashboard.putNumber("BRA ABS", m_backRight.Encoder.getAbsolutePosition());
        SmartDashboard.putNumber("BLA ABS", m_backLeft.Encoder.getAbsolutePosition());
        // SmartDashboard.putNumber("absolute encoder heckin value",
        // m_frontRight.trueEncoderOffset);
        SmartDashboard.putString("hecking auto", this.m_pose.toString());
        SmartDashboard.putString("State angle", m_frontRight.getState().angle.toString());
        /*
        SmartDashboard.putNumber("FRA-ActualPS",
        m_frontRight.getTurnEncoderDistance());
        SmartDashboard.putNumber("FLA-ActualPS",
        m_frontLeft.getTurnEncoderDistance());
        SmartDashboard.putNumber("BRA-ActualPS",
        m_backRight.getTurnEncoderDistance());
        SmartDashboard.putNumber("BLA-ActualPS",
        m_backLeft.getTurnEncoderDistance());
        SmartDashboard.putNumber("FRA-Actual",
        (FREncoder.getAbsolutePosition()-m_frontRight.angleOffset)*2*Math.PI);
        SmartDashboard.putNumber("FLA-Actual",
        (FLEncoder.getAbsolutePosition()-m_frontLeft.angleOffset)*2*Math.PI);
        SmartDashboard.putNumber("BRA-Actual",
        (BREncoder.getAbsolutePosition()-m_backRight.angleOffset)*2*Math.PI);
        SmartDashboard.putNumber("BLA-Actual",
        (BLEncoder.getAbsolutePosition()-m_backLeft.angleOffset)*2*Math.PI);
        
        SmartDashboard.putNumber("FRA-Setpoint", m_frontRight.getAngleSetpoint());
        SmartDashboard.putNumber("FLA-Setpoint", m_frontLeft.getAngleSetpoint());
        SmartDashboard.putNumber("BRA-Setpoint", m_backRight.getAngleSetpoint());
        SmartDashboard.putNumber("BLA-Setpoint", m_backLeft.getAngleSetpoint());
         */
        SmartDashboard.putNumber("P-value", RobotContainer.pEditor + 5);
        SmartDashboard.putNumber("D-value", RobotContainer.dEditor + .7);
        SmartDashboard.putNumber("FRD-Actual",
            m_frontRight.getDriveEncoderDistance() - m_frontRight.getDriveSetpoint());
        SmartDashboard.putNumber("FLD-Actual", m_frontLeft.getDriveEncoderDistance() - m_frontLeft.getDriveSetpoint());
        SmartDashboard.putNumber("BRD-Actual", m_backRight.getDriveEncoderDistance() - m_backRight.getDriveSetpoint());
        SmartDashboard.putNumber("BLD-Actual", m_backLeft.getDriveEncoderDistance() - m_backLeft.getDriveSetpoint());

        /*
        SmartDashboard.putNumber("FRD-Setpoint", m_frontRight.getDriveSetpoint());
        SmartDashboard.putNumber("FLD-Setpoint", m_frontLeft.getDriveSetpoint());
        SmartDashboard.putNumber("BRD-Setpoint", m_backRight.getDriveSetpoint());
        SmartDashboard.putNumber("BLD-Setpoint", m_backLeft.getDriveSetpoint());
         */

    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

        // PID Tuning from Shuffleboard
        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

        // ====================================================================
        // PID EDITING
        // ====================================================================
        GenericEntry pFL = driveTab.add("FL P", DriveConstants.kFLTurnKp)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(0, 0)
                        .getEntry();

        GenericEntry iFL = driveTab.add("FL I", DriveConstants.kFLTurnKi)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(1, 0)
                        .getEntry();

        GenericEntry dFL = driveTab.add("FL D", DriveConstants.kFLTurnKd)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(2, 0)
                        .getEntry();

        GenericEntry pFR = driveTab.add("FR P", DriveConstants.kFRTurnKp)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(0, 1)
                        .getEntry();

        GenericEntry iFR = driveTab.add("FR I", DriveConstants.kFRTurnKi)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(1, 1)
                        .getEntry();

        GenericEntry dFR = driveTab.add("FR D", DriveConstants.kFRTurnKd)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(2, 1)
                        .getEntry();

        GenericEntry pBL = driveTab.add("BL P", DriveConstants.kBLTurnKp)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(0, 2)
                        .getEntry();

        GenericEntry iBL = driveTab.add("BL I", DriveConstants.kBLTurnKi)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(1, 2)
                        .getEntry();

        GenericEntry dBL = driveTab.add("BL D", DriveConstants.kBLTurnKd)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(2, 2)
                        .getEntry();

        GenericEntry pBR = driveTab.add("BR P", DriveConstants.kBRTurnKp)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(0, 3)
                        .getEntry();

        GenericEntry iBR = driveTab.add("BR I", DriveConstants.kBRTurnKi)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(1, 3)
                        .getEntry();

        GenericEntry dBR = driveTab.add("BR D", DriveConstants.kBRTurnKd)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withProperties(Map.of("min", 0, "max", 10))
                        .withPosition(2, 3)
                        .getEntry();
        RobotContainer robotContainer;

        public static SwerveModule // Create the Swerve Modules
        frontLeft = new SwerveModule(
                        DriveConstants.kFrontLeftDriveID,
                        DriveConstants.kFrontLeftTurningID,
                        DriveConstants.FLEncoderOffset,
                        FLEncoder,
                        DriveConstants.kFLTurnKp, DriveConstants.kFLTurnKi, DriveConstants.kFLTurnKd),
                        frontRight = new SwerveModule(
                                        DriveConstants.kFrontRightDriveID,
                                        DriveConstants.kFrontRightTurningID,
                                        DriveConstants.FREncoderOffset,
                                        FREncoder,
                                        DriveConstants.kFRTurnKp, DriveConstants.kFRTurnKi, DriveConstants.kFRTurnKd),
                        backLeft = new SwerveModule(
                                        DriveConstants.kBackLeftDriveID,
                                        DriveConstants.kBackLeftTurningID,
                                        DriveConstants.BLEncoderOffset,
                                        BLEncoder,
                                        DriveConstants.kBLTurnKp, DriveConstants.kBLTurnKi, DriveConstants.kBLTurnKd),
                        backRight = new SwerveModule(
                                        DriveConstants.kBackRightDriveID,
                                        DriveConstants.kBackRightTurningID,
                                        DriveConstants.BREncoderOffset,
                                        BREncoder,
                                        DriveConstants.kBRTurnKp, DriveConstants.kBRTurnKi, DriveConstants.kBRTurnKd);

        public static SwerveModulePosition[] positions = {
                        DrivetrainSubsystem.frontLeft.getPosition(),
                        DrivetrainSubsystem.frontRight.getPosition(),
                        DrivetrainSubsystem.backLeft.getPosition(),
                        DrivetrainSubsystem.backRight.getPosition()
        };
        // 0.306
        // 0.257
        private Translation2d // Create the locations of the wheels
        frontRightLocation = new Translation2d(0.257, -0.306),
                        // 0.306
                        // 0.257
                        backRightLocation = new Translation2d(0.257, 0.306),
                        frontLeftLocation = new Translation2d(-0.257, -0.306),
                        backLeftLocation = new Translation2d(-0.257, 0.306);

        public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        public final AHRS gyro = new AHRS(Port.kMXP);
        public double gyroOffset = gyro.getAngle();

        public Pose2d pose = new Pose2d(0, 0, getGyro());
        public final SwerveDriveOdometry odometry;

        public DrivetrainSubsystem() {

                gyro.calibrate(); // this probably isn't necessary but idk
                this.resetGyro(); // this however, is necessary ish
                this.odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), positions, pose);
                odometry.resetPosition(this.getGyro(), positions, pose);
        }

        public Pose2d getPositionPose2d() {
                return odometry.getPoseMeters();
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
                        var swerveModuleStates = kinematics.toSwerveModuleStates(
                                        fieldRelative
                                                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                                        this.getGyro())
                                                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
                        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
                        frontLeft.setDesiredState(swerveModuleStates[0]);
                        frontRight.setDesiredState(swerveModuleStates[1]);
                        backLeft.setDesiredState(swerveModuleStates[2]);
                        backRight.setDesiredState(swerveModuleStates[3]);
                } else {
                        frontLeft.driveVoltage(0);
                        backRight.driveVoltage(0);
                        backLeft.driveVoltage(0);
                        frontRight.driveVoltage(0);
                }
        }

        /*
         * public void fakeConverter(SwerveModuleState[] param){
         * for (int i = 0; i < 4; i++) {
         * param = swerveModuleStates;
         * SwerveModulePosition[] output = swerveModuleFakeStates;
         * swerveModuleStates[i].speedMetersPerSecond =
         * swerveModuleFakeStates[i].distanceMeters;
         * swerveModuleFakeStates[i].angle = swerveModuleFakeStates[i].angle;
         * }
         * }
         */
        public void resetPose(SwerveModulePosition[] positions, Pose2d pose) {
                // odometry.resetPosition(null, swerveModuleFakeStates, pose);
                odometry.resetPosition(this.getGyro(), positions, pose);
        }

        public void pleaseGodLetThisWork(SwerveModuleState Wheel1, SwerveModuleState Wheel2, SwerveModuleState Wheel3,
                        SwerveModuleState Wheel4) {
                frontLeft.setDesiredState(Wheel1);
                frontRight.setDesiredState(Wheel2);
                backLeft.setDesiredState(Wheel3);
                backRight.setDesiredState(Wheel4);
        }

        public void driveVoltage(double voltage) {
                frontLeft.driveVoltage(voltage);
                frontRight.driveVoltage(voltage);
                backLeft.driveVoltage(voltage);
                backRight.driveVoltage(voltage);
        }

        /** Returns the rotation relative to the last resetGyro() called */
        public Rotation2d getInitGyro() {
                Rotation2d gyro = new Rotation2d((this.gyro.getAngle() - gyroOffset - 90) * (Math.PI) / 180);
                return gyro;
        }

        public Rotation2d getGyro() {
                Rotation2d gyro = new Rotation2d((this.gyro.getAngle() - gyroOffset) * (Math.PI) / 180);
                return gyro;
        }

        public double getHeading() {
                return gyro.getAngle();
        }

        /**
         * Sets the offset used in
         * {@link frc.robot.subsystems.DrivetrainSubsystem#getGyro()}
         */
        public void resetGyro() {
                gyroOffset = gyro.getAngle();
        }

        public SwerveModulePosition[] getPosition() {
                return positions;
        }

        @Override
        public void periodic() {

                Rotation2d gyroAngle = getGyro();
                pose = odometry.update(gyroAngle, new SwerveModulePosition[] {
                                DrivetrainSubsystem.frontLeft.getPosition(),
                                DrivetrainSubsystem.frontRight.getPosition(),
                                DrivetrainSubsystem.backLeft.getPosition(),
                                DrivetrainSubsystem.backRight.getPosition()
                });

                /*
                 * frontLeft.updateTurnPID(
                 * pFL.getDouble(DriveConstants.kFLTurnKp),
                 * dFL.getDouble(DriveConstants.kFLTurnKi),
                 * dFL.getDouble(DriveConstants.kFLTurnKd));
                 * frontRight.updateTurnPID(
                 * pFR.getDouble(DriveConstants.kFRTurnKp),
                 * dFR.getDouble(DriveConstants.kFRTurnKi),
                 * dFR.getDouble(DriveConstants.kFRTurnKd));
                 * backLeft.updateTurnPID(
                 * pBL.getDouble(DriveConstants.kBLTurnKp),
                 * dBL.getDouble(DriveConstants.kBLTurnKi),
                 * dBL.getDouble(DriveConstants.kBLTurnKd));
                 * backRight.updateTurnPID(
                 * pBR.getDouble(DriveConstants.kBRTurnKp),
                 * dBR.getDouble(DriveConstants.kBRTurnKi),
                 * dBR.getDouble(DriveConstants.kBRTurnKd));
                 */

                // frontLeft.collectEncoderSample();
                // frontRight.collectEncoderSample();
                // backLeft.collectEncoderSample();
                // backRight.collectEncoderSample();

                // SmartDashboard.putNumber("FL Raw Encoder", frontLeft.getTurnEncoderRaw());
                // SmartDashboard.putNumber("FR Raw Encoder", frontRight.getTurnEncoderRaw());
                // SmartDashboard.putNumber("BL Raw Encoder", backLeft.getTurnEncoderRaw());
                // SmartDashboard.putNumber("BR Raw Encoder", backRight.getTurnEncoderRaw());

                // SmartDashboard.putNumber("FL Motor Angle",
                // frontLeft.getTurnEncoderDistance());
                // SmartDashboard.putNumber("FL Abs Angle",
                // frontLeft.getBetterTurnEncoderDistance());

                // SmartDashboard.putNumber("FL Turn Setpoint", frontLeft.getAngleSetpoint());
                // SmartDashboard.putNumber("FR Turn Setpoint",
                // frontRight.getAngleSetpoint());
                // SmartDashboard.putNumber("BL Turn Setpoint", backLeft.getAngleSetpoint());
                // SmartDashboard.putNumber("BR Turn Setpoint", backRight.getAngleSetpoint());
                // SmartDashboard.putString("Gyro", getGyro().toString());
                // SmartDashboard.putNumber("normal gyro", getHeading());
                // SmartDashboard.putString("GyroFake", this.getGyro().toString());
                // SmartDashboard.putNumber("Gyro Angle", this.getHeading());
                // SmartDashboard.putString("Gyro Offset", this.gyroOffset.toString());

                // SmartDashboard.putNumber("testy boiFL", (frontLeft.trueEncoderOffset));
                // SmartDashboard.putNumber("testy boiFR", (frontRight.trueEncoderOffset));
                // SmartDashboard.putNumber("testy boiBL", (backLeft.trueEncoderOffset));
                // SmartDashboard.putNumber("testy boiBR", (backRight.trueEncoderOffset));
                // SmartDashboard.putNumber("FR Pose", frontRight.getTurnEncoderDistance());
                // SmartDashboard.putNumber("BR Pose", backRight.getTurnEncoderDistance());
                // SmartDashboard.putNumber("FL Pose", frontLeft.getTurnEncoderDistance());
                // SmartDashboard.putNumber("BL Pose", backLeft.getTurnEncoderDistance());
                // SmartDashboard.putNumber("FR ABS", frontRight.getAbsTurnEncoder());
                // SmartDashboard.putNumber("FL ABS", frontLeft.getAbsTurnEncoder());
                // SmartDashboard.putNumber("BR ABS", backRight.getAbsTurnEncoder());
                // SmartDashboard.putNumber("BL ABS", backLeft.getAbsTurnEncoder());

                // SmartDashboard.putNumber("FR Offset", frontRight.getOffset());
                // SmartDashboard.putNumber("FL Offset", frontLeft.getOffset());
                // SmartDashboard.putNumber("BR Offset", backRight.getOffset());
                // SmartDashboard.putNumber("BL Offset", backLeft.getOffset());

                // SmartDashboard.putNumber("BR Abs Position Error",
                // backRight.getTurningPID().getPositionError());
                // SmartDashboard.putNumber("FR Abs Position Error",
                // frontRight.getTurningPID().getPositionError());
                // SmartDashboard.putNumber("BL Abs Position Error",
                // backLeft.getTurningPID().getPositionError());
                // SmartDashboard.putNumber("FL Abs Position Error",
                // frontLeft.getTurningPID().getPositionError());

                // SmartDashboard.putNumber("FL Error + Encoder",
                // frontLeft.getTurnEncoderDistance() +
                // frontLeft.getTurningPID().getPositionError());
                // SmartDashboard.putNumber("FR Error + Encoder",
                // frontRight.getTurnEncoderDistance() +
                // frontRight.getTurningPID().getPositionError());
                // SmartDashboard.putNumber("BL Error + Encoder",
                // backLeft.getTurnEncoderDistance() +
                // backLeft.getTurningPID().getPositionError());
                // SmartDashboard.putNumber("BR Error + Encoder",
                // backRight.getTurnEncoderDistance() +
                // backRight.getTurningPID().getPositionError());
                // SmartDashboard.putNumber("absolute encoder heckin value",
                // frontRight.trueEncoderOffset);

                // SmartDashboard.putString("hecking auto", this.pose.toString());
                // SmartDashboard.putString("State angle",
                // frontRight.getState().angle.toString());

                /*
                 * SmartDashboard.putNumber("FRA-ActualPS",
                 * frontRight.getTurnEncoderDistance());
                 * SmartDashboard.putNumber("FLA-ActualPS",
                 * frontLeft.getTurnEncoderDistance());
                 * SmartDashboard.putNumber("BRA-ActualPS",
                 * backRight.getTurnEncoderDistance());
                 * SmartDashboard.putNumber("BLA-ActualPS",
                 * backLeft.getTurnEncoderDistance());
                 * SmartDashboard.putNumber("FRA-Actual",
                 * (FREncoder.getAbsolutePosition()-frontRight.angleOffset)*2*Math.PI);
                 * SmartDashboard.putNumber("FLA-Actual",
                 * (FLEncoder.getAbsolutePosition()-frontLeft.angleOffset)*2*Math.PI);
                 * SmartDashboard.putNumber("BRA-Actual",
                 * (BREncoder.getAbsolutePosition()-backRight.angleOffset)*2*Math.PI);
                 * SmartDashboard.putNumber("BLA-Actual",
                 * (BLEncoder.getAbsolutePosition()-backLeft.angleOffset)*2*Math.PI);
                 * 
                 * SmartDashboard.putNumber("FRA-Setpoint", frontRight.getAngleSetpoint());
                 * SmartDashboard.putNumber("FLA-Setpoint", frontLeft.getAngleSetpoint());
                 * SmartDashboard.putNumber("BRA-Setpoint", backRight.getAngleSetpoint());
                 * SmartDashboard.putNumber("BLA-Setpoint", backLeft.getAngleSetpoint());
                 */
                // SmartDashboard.putNumber("P-value", RobotContainer.pEditor + 5);
                // SmartDashboard.putNumber("D-value", RobotContainer.dEditor + .7);

                // SmartDashboard.putNumber("FRD-Actual",
                // frontRight.getDriveEncoderDistance() - frontRight.getDriveSetpoint());
                // SmartDashboard.putNumber("FLD-Actual", frontLeft.getDriveEncoderDistance()
                // - frontLeft.getDriveSetpoint());
                // SmartDashboard.putNumber("BRD-Actual", backRight.getDriveEncoderDistance()
                // - backRight.getDriveSetpoint());
                // SmartDashboard.putNumber("BLD-Actual", backLeft.getDriveEncoderDistance() -
                // backLeft.getDriveSetpoint());

                /*
                 * SmartDashboard.putNumber("FRD-Setpoint", frontRight.getDriveSetpoint());
                 * SmartDashboard.putNumber("FLD-Setpoint", frontLeft.getDriveSetpoint());
                 * SmartDashboard.putNumber("BRD-Setpoint", backRight.getDriveSetpoint());
                 * SmartDashboard.putNumber("BLD-Setpoint", backLeft.getDriveSetpoint());
                 */

                SmartDashboard.putNumber("Yaw", gyro.getYaw());
                SmartDashboard.putNumber("Pitch", gyro.getPitch());
                SmartDashboard.putNumber("Roll", gyro.getRoll());

        }
}
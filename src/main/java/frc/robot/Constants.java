// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final class DriveConstants {
                public static final int kFrontLeftDriveID = 15;
                public static final int kFrontRightDriveID = 14;
                public static final int kBackLeftDriveID = 12;
                public static final int kBackRightDriveID = 9;
                public static final int kFrontLeftTurningID = 16;
                public static final int kFrontRightTurningID = 13;
                public static final int kBackLeftTurningID = 11;
                public static final int kBackRightTurningID = 10;

                public static final double kMaxSpeed = 5; // should be 5 meters per second, 2 rn so ian keeps his ankles

                public static final double kMaxAngularSpeed = 2 * Math.PI; // is currently 1, will be 1/2 eventually
                // (don't change)
                public static final double kMaxAngularAcceleration = Math.PI;

                public static final double kTurnKp = 7;
                public static final double kTurnKi = 12;
                public static final double kTurnKd = .1;

                // public static final double kFLTurnKp = 7;
                // public static final double kFLTurnKi = 12;
                // public static final double kFLTurnKd = 0.1;

                // public static final double kFRTurnKp = 7;
                // public static final double kFRTurnKi = 12;
                // public static final double kFRTurnKd = 0.1;

                // public static final double kBLTurnKp = 7;
                // public static final double kBLTurnKi = 12;
                // public static final double kBLTurnKd = 0.1;

                // public static final double kBRTurnKp = 7;
                // public static final double kBRTurnKi = 12;
                // public static final double kBRTurnKd = 0.1;

                public static final double kFLTurnKp = 7;
                public static final double kFLTurnKi = 5;
                public static final double kFLTurnKd = 0.16;

                public static final double kFRTurnKp = 7;
                public static final double kFRTurnKi = 4;
                public static final double kFRTurnKd = 0.13;

                public static final double kBLTurnKp = 7;
                public static final double kBLTurnKi = 2;
                public static final double kBLTurnKd = 0.1;

                public static final double kBRTurnKp = 8;
                public static final double kBRTurnKi = 0;
                public static final double kBRTurnKd = 0;

                // next lines are for the Swerve Module object
                public static final double BLEncoderOffset = 0.4330 - .25;
                public static final double FLEncoderOffset = 0.40145 - .25;
                public static final double BREncoderOffset = 0.66424 - .25;
                public static final double FREncoderOffset = 0.909868 - .25;
                public static final double kWheelRadius = 0.0508; // meters
                public static final double kEncoderResolution = 2048;

                public static final double kDriveGearRatio = 6.54;
                public static final double kTurnGearRatio = 15.43;

                public static final int FREncoderID = 0, FLEncoderID = 3, BREncoderID = 1, BLEncoderID = 2;

                public static final double kTurnEncoderConstant = 2 * Math.PI / (kTurnGearRatio * kEncoderResolution);
        }

        public static final class FieldConstants {
                public static final Rotation2d REDANGLE_ROTATION2D = new Rotation2d(0);
                public static final Pose2d[] REDPOLES_POSE2DS = { new Pose2d(0, 0, REDANGLE_ROTATION2D),
                                new Pose2d(0, 0, REDANGLE_ROTATION2D),
                                new Pose2d(0, 0, REDANGLE_ROTATION2D),
                                new Pose2d(0, 0, REDANGLE_ROTATION2D),
                                new Pose2d(0, 0, REDANGLE_ROTATION2D),
                                new Pose2d(0, 0, REDANGLE_ROTATION2D),
                                new Pose2d(0, 0, REDANGLE_ROTATION2D),
                                new Pose2d(0, 0, REDANGLE_ROTATION2D),
                                new Pose2d(0, 0, REDANGLE_ROTATION2D) };
                public static final Rotation2d BLUE_ROTATION2D = new Rotation2d(0);
                public static final Pose2d[] BLUEPOLES_POSE2DS = { new Pose2d(0, 0, BLUE_ROTATION2D),
                                new Pose2d(0, 0, BLUE_ROTATION2D),
                                new Pose2d(0, 0, BLUE_ROTATION2D),
                                new Pose2d(0, 0, BLUE_ROTATION2D),
                                new Pose2d(0, 0, BLUE_ROTATION2D),
                                new Pose2d(0, 0, BLUE_ROTATION2D),
                                new Pose2d(0, 0, BLUE_ROTATION2D),
                                new Pose2d(0, 0, BLUE_ROTATION2D),
                                new Pose2d(0, 0, BLUE_ROTATION2D) };
        }

        public static final class IntakeConstants {
                public static final int kIntakeID = 5;
                public static final int kVoltageLimit = -1;
        }

        public static final class SlideConstants {
                // ===============================================================
                // SLIDE INTAKE SETPOINTS
                // ===============================================================

                public static final double[] coneIntakeSetpoints = {
                                3.3913, // 0 - human player intake
                                3.7496, // 1 - floor intake (flipped)
                                20.039 // 2 - floor intake (upright)

                };

                public static final double[] cubeIntakeSetpoints = {
                                0, // 0 - floor intake
                                1.8917, // 1 - human player
                                1.8917
                };

                // ===============================================================
                // SLIDE OUTTAKE SETPOINTS
                // ===============================================================

                public static final double[] cubeOuttakeSetpoints = {
                                0, // 0 - ground level (hybrid)
                                25.267, // 1 - first platform
                                46.1483 // 2 - second platform (highest)
                };

                public static final double[] coneOuttakeSetpoints = {
                                16.1876, // 0 - ground level (hybrid)
                                53.9773, // 1 - first pole
                                59.3297 // 2 - second pole (highest)
                };

                // ===============================================================

                public static final double kP = 0.17;
                public static final double kI = 0.01;
                public static final double kD = 0.003;

                public static final double maxVelocity = 120; // inches per second
                public static final double maxAcceleration = 180; // inches per second squared

                public static final double maxEncoderTicks = 207000;

                public static final int kLSlideID = 2;
                public static final int kRSlideID = 4;
                public static final double kSlideConstant = 1 / 16384;

                public static final int EncoderId = -1;
                public static final int EncoderOffset = -1;
                public static final double CurrentThreshold = 30;
                public static final double CurrentDeadband = 0;

                public static final int levelZeroHeight = -1, levelOneHeight = -1, levelTwoHeight = -1,
                                levelThreeHeight = -1;

                public static final int kBottomLimitSwitchId = 4;
                public static final int kTopLimitSwitchId = 6;

                public static final double kWinchRadius = 0.75;
                public static final double kGearRatio = 8;
                public static final double kNumOfStages = 2;

                public static double rawUnitsToInches(double raw) {
                        return ((raw / 8) / 2048) * (Math.PI * kWinchRadius) * kNumOfStages;
                }
        }

        public static final class WristConstants {
                public static final int kWristIDLeft = 0, kWristIDRight = 1;
                public static final int kEncoderID = 5;

                public static final double kWristSafePosition = -135, kConeSafePosition = -110;

                public static final double kEncoderOffset = 0.44;

                // ===============================================================
                // WRIST INTAKE SETPOINTS
                // ===============================================================

                public static final double[] coneIntakeSetpoints = {
                                -41.9187, // 0 - human player intake
                                40.7582, // 1 - floor intake (flipped)
                                61.9327 // 2 - floor intake (upright)
                };

                public static final double[] cubeIntakeSetpoints = {
                                -12.1624, // 0 - floor intake
                                -118.5223, // 1 - human player
                                -118.5223 // 2 - human player
                };

                // ===============================================================
                // WRIST OUTTAKE SETPOINTS
                // ===============================================================

                public static final double[] cubeOuttakeSetpoints = {
                                -90, // 0 - ground level (hybrid)
                                -79.666, // 1 - first platform
                                -110, // 2 - second platform (highest)
                };

                public static final double[] coneOuttakeSetpoints = {
                                0, // 0 - ground level (hybrid)
                                49.9116, // 1 - first pole
                                20, // 2 - second pole (highest)
                };

                // ===============================================================

                public static final double kP = 0.02;
                public static final double kI = 0.0;
                public static final double kD = 0;

                public static final double maxVelocity = 40; // degrees per second
                public static final double maxAcceleration = 80; // degrees per second squared

                public static final double kGearRatio = 128;

                public static double convertTicksToRadians(double ticks, double offset) {
                        double revolutions = (ticks / kGearRatio) / DriveConstants.kEncoderResolution;
                        return (revolutions * 360) + offset;
                }
        }

        // Have to add mirrored trajectories if alliance is switched

        public static final class VisionConstants {
                private static final double fieldLength = Units.inchesToMeters((54 * 12) + 3.25);
                private static final double fieldWidth = Units.inchesToMeters((26 * 12) + 3.5);
                static double wallY1 = Units.inchesToMeters(351);
                static double wallY2 = Units.inchesToMeters(97);
                public static final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(
                                List.of(
                                                new AprilTag(1,
                                                                new Pose3d(Units.inchesToMeters(120.25 + 300), wallY1,
                                                                                Units.inchesToMeters(23.75),
                                                                                new Rotation3d(VecBuilder.fill(0, 0, 1),
                                                                                                Units.degreesToRadians(
                                                                                                                270)))),
                                                new AprilTag(2,
                                                                new Pose3d(0, 0, Units.inchesToMeters(51.5),
                                                                                new Rotation3d(VecBuilder.fill(0, 0, 1),
                                                                                                Units.degreesToRadians(
                                                                                                                90)))),
                                                new AprilTag(3,
                                                                new Pose3d(Units.inchesToMeters(214.375 + 300),
                                                                                Units.inchesToMeters(29.25),
                                                                                Units.inchesToMeters(26.25),
                                                                                new Rotation3d(VecBuilder.fill(0, 0, 1),
                                                                                                Units.degreesToRadians(
                                                                                                                180)))),
                                                new AprilTag(5,
                                                                new Pose3d(Units.inchesToMeters(208.125), wallY1,
                                                                                Units.inchesToMeters(44),
                                                                                new Rotation3d(VecBuilder.fill(0, 0, 1),
                                                                                                Units.degreesToRadians(
                                                                                                                270)))),
                                                new AprilTag(6,
                                                                new Pose3d(0, 0, Units.inchesToMeters(57.0625),
                                                                                new Rotation3d(VecBuilder.fill(0, 0, 1),
                                                                                                Units.degreesToRadians(
                                                                                                                90)))),
                                                new AprilTag(7,
                                                                new Pose3d(Units.inchesToMeters(30.5), wallY1,
                                                                                Units.inchesToMeters(43),
                                                                                new Rotation3d(VecBuilder.fill(0, 0, 1),
                                                                                                Units.degreesToRadians(
                                                                                                                270))))),
                                VisionConstants.fieldLength, VisionConstants.fieldWidth);

                public static final Transform3d robotToCamera = new Transform3d(
                                new Translation3d(Units.inchesToMeters(13.125), 0, Units.inchesToMeters(6.125)),
                                new Rotation3d(0, 0, 0));

                public static final double camDiagFOV = 95.0;
                public static final double camPitch = robotToCamera.getRotation().getX();
                public static final double camHeight = robotToCamera.getTranslation().getZ();
                public static final double maxLEDRange = 0;
                public static final int camResolutionWidth = 1280, camResolutionHeight = 720;
                public static final double minTargetArea = 10;
                public static final String kCameraName = "USB Camera 0";
        }

        public static final class LightConstants {
                public static final int lightstrip1Port = 9;
                public static final int lightstrip1Length = 149;
        }
}

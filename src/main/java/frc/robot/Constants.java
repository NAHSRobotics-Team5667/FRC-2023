// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final int kFrontRightDriveID = 9;
        public static final int kBackLeftDriveID = 14;
        public static final int kBackRightDriveID = 12;
        public static final int kFrontLeftTurningID = 16;
        public static final int kFrontRightTurningID = 10;
        public static final int kBackLeftTurningID = 13;
        public static final int kBackRightTurningID = 11;

        public static final double kMaxSpeed = 2; // should be 5 meters per second, 2 rn so ian keeps his ankles
        public static final double kMaxAngularSpeed = 2 * Math.PI; // is currently 1, will be 1/2 eventually (don't change)

        // next lines are for the Swerve Module object
        public static final double kWheelRadius = 0.0508; // meters
        public static final double kEncoderResolution = 2048;

        public static final double kDriveGearRatio = 6.54;
        public static final double kTurnGearRatio = 15.43;

        public static final int FREncoderID = 2;
        public static final int FLEncoderID = 0;
        public static final int BREncoderID = 3;
        public static final int BLEncoderID = 1;

        public static final double FREncoderOffset = 0.412877;
        public static final double FLEncoderOffset = 0.4076069;
        public static final double BREncoderOffset = .182498;
        public static final double BLEncoderOffset = 0.406126;
        public static final double kTurnEncoderConstant = 2 * Math.PI / (kTurnGearRatio * kEncoderResolution);

    }

    public static final class ClawConstants {
        public static final int kClawID = -1;
    }

    public static final class SlideConstants {
        public static final int kLSlideID = -1;
        public static final int kRSlideID = -1;
        public static final int kTiltID = -1;
        public static final int kSlideConstant = -1; //TODO: Find gear ratio
    }

}

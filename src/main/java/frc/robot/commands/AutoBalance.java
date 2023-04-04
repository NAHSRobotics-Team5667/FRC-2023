package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {
    RobotContainer robotContainer;
    DrivetrainSubsystem drive;
    double multiplierPitch = 2, multiplierRoll = 2, clock = 0, delay, x = 0, y = 0;
    boolean done = false, slowTheHeckDownRoll = false, slowTheHeckDownPitch = false;
    AHRS gyro;

    public AutoBalance(RobotContainer robotContainer, DrivetrainSubsystem drive, double delay) {
        this.robotContainer = robotContainer;
        this.drive = drive;
        this.delay = delay;
        this.gyro = drive.gyro;
        addRequirements(drive);
    }

    // we want pitch and roll... if roll is off move side to side, if pitch is off
    // move forward back
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        x = 0;
        y = 0;

        if (delay > clock && delay != 0) {
            clock += .02;
        } else {
            // gyro values may be but
            float roll = gyro.getRoll(), pitch = gyro.getPitch();
            SmartDashboard.putNumber("roll", roll);
            SmartDashboard.putNumber("pitch", pitch);
            if (Math.abs(roll) > 10 && !slowTheHeckDownRoll) {
                multiplierRoll = 2;
            } else {
                multiplierRoll = 1.4;
                slowTheHeckDownRoll = true;
            }

            if (Math.abs(pitch) > 10 && !slowTheHeckDownPitch) {
                multiplierPitch = 2;
            } else {
                slowTheHeckDownPitch = true;
                multiplierPitch = 1.4;
            }

            if (roll > 10) {
                x = .45 * multiplierRoll;
            } else if (roll < -10) {
                x = -.45 * multiplierRoll;
            }

            if (pitch > 10) {
                y = -.45 * multiplierPitch;
            } else if (pitch < -10) {
                y = .45 * multiplierPitch;
            }
            // // if (Math.abs(pitch) < 5 && Math.abs(roll) < 5) {
            // done = true;
            // }

            SmartDashboard.putNumber("unknown", 222);
        }
        drive.drive(x, y, 0, false); // what
    }

    public boolean isFinished() {
        return done;
    }
}

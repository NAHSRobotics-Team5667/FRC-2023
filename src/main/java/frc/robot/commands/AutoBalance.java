package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {
    RobotContainer robotContainer;
    DrivetrainSubsystem drive;
    boolean done = false;
    double multiplierPitch = 2, multiplierRoll = 2;
    boolean slowTheHeckDownRoll = false, slowTheHeckDownPitch = false;
    AHRS gyro;

    public AutoBalance(RobotContainer robotContainer, DrivetrainSubsystem drive) {
        this.robotContainer = robotContainer;
        this.drive = drive;
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
        double x = 0;
        double y = 0;
        // gyro values may be but
        float roll = gyro.getRoll();
        float pitch = gyro.getPitch();
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
        drive.drive(x, y, 0, false);
        SmartDashboard.putNumber("unknown", 222); // what
    }

    public boolean isFinished() {
        return done;
    }
}

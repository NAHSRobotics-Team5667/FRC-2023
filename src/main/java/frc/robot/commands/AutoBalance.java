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

    AHRS gyro;

    public AutoBalance(RobotContainer robotContainer, DrivetrainSubsystem drive) {
        this.robotContainer = robotContainer;
        this.drive = drive;
        this.gyro = drive.gyro;
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

        if (roll > 5) {
            x = -.5;
        } else if (roll < -5) {
            x = .5;
        }
        if (pitch > 5) {
            y = -.5;
        } else if (pitch < -5) {
            y = .5;
        }
        if (Math.abs(pitch) < 5 && Math.abs(roll) < 5) {
            done = true;
        }
        drive.drive(x, y, 0, false);
    }

    public boolean isFinished() {
        return done;
    }
}

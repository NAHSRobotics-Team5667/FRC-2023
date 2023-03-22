// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

    // public int bumperPos = 0;
    public WPI_TalonFX wristMotorFirst, wristMotorSecond;
    private PIDController wristPID = new PIDController(.02, 0, 0);
    private DutyCycleEncoder encoder;
    private double angleOffset = 0;
    private int counter = 0;
    private RobotContainer robotContainer;

    ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");

    // ====================================================================
    // PID EDITING
    // ====================================================================
    GenericEntry p = wristTab.add("Wrist P", WristConstants.kP)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", 0, "max", 10))
            .withPosition(0, 0)
            .getEntry();

    GenericEntry i = wristTab.add("Wrist I", WristConstants.kI)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", 0, "max", 10))
            .withPosition(1, 0)
            .getEntry();

    GenericEntry d = wristTab.add("Wrist D", WristConstants.kD)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", 0, "max", 10))
            .withPosition(2, 0)
            .getEntry();
    // ====================================================================

    /** Creates a new WristSubsystem. */
    public WristSubsystem(RobotContainer robotContainer) {
        encoder = new DutyCycleEncoder(5);
        wristPID.setTolerance(2.5);

        wristMotorFirst = new WPI_TalonFX(Constants.WristConstants.kWristIDLeft);
        wristMotorSecond = new WPI_TalonFX(Constants.WristConstants.kWristIDRight);

        wristMotorFirst.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        wristMotorFirst.setSelectedSensorPosition(0);

        wristMotorSecond.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        wristMotorSecond.setSelectedSensorPosition(0);

        wristMotorFirst.setInverted(true);

        this.robotContainer = robotContainer;

        // angleOffset = (getEncoder() - WristConstants.kEncoderOffset) * 360;
    }

    public void setWrist(double percentOutput) {
        if (getAngleDegrees() < -125) {
            percentOutput = Math.max(percentOutput, 0);
        } else if (getAngleDegrees() > 90) {
            percentOutput = Math.min(percentOutput, 0);
        }

        wristMotorFirst.set(ControlMode.PercentOutput, percentOutput);
        wristMotorSecond.set(ControlMode.PercentOutput, percentOutput);
    }

    public double getEncoder() {
        return encoder.getAbsolutePosition();
    }

    public void setPosition(double position) {
        double output = MathUtil.clamp(wristPID.calculate(getAngleDegrees(), position), -0.4, 0.4);
        setWrist(output);
    }

    // public int getBumperPos() {
    // return bumperPos;
    // }

    // public void setBumperPos(int bumperPos) {
    // this.bumperPos = bumperPos;
    // }

    public double getPosition() {
        return (wristMotorFirst.getSelectedSensorPosition() + wristMotorSecond.getSelectedSensorPosition()) / 2;
    }

    public double getDriveRate() {
        return wristMotorFirst.getSelectedSensorVelocity();
    }

    public double pidError() {
        return wristPID.getPositionError();
    }

    public void updatePID(double kP, double kI, double kD) {
        wristPID.setP(kP);
        wristPID.setI(kI);
        wristPID.setD(kD);
    }

    // make these dependent on bumperPos, array of setPoints
    public void coneIntakeAngled() { // TODO: this method doesnt do shit
        @SuppressWarnings("unused")
        double currentPosition = getPosition();
        // double outputWrist =

        // wristPID.calculate(currentPosition,
        // Constants.WristConstants.kWristConeIntakeSetpoint);
        // double wristFeedForward = wristFeedForward.calculate(getDriveRate());
        // wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
        // wristMotorSecond.setVoltage(outputWrist + wristFeedForward);

        // Wrist Angled for Cone intake
    }

    public double getAngleDegrees() {
        return WristConstants.convertTicksToRadians(wristMotorFirst.getSelectedSensorPosition(), angleOffset);
    }

    @Override
    public void periodic() {
        if (counter <= 50) {
            counter++;
        }

        // counter++;

        if (counter % 50 == 0) {
            angleOffset = (getEncoder() - WristConstants.kEncoderOffset) * 360;
        }

        SmartDashboard.putNumber("Wrist Setpoint", wristPID.getSetpoint());

        SmartDashboard.putNumber("Position Level", robotContainer.getPositionLevel());
        SmartDashboard.putNumber("Wrist Encoder", getEncoder());
        SmartDashboard.putNumber("Wrist Angle", getAngleDegrees());
        SmartDashboard.putNumber("Wrist Error", pidError());
        SmartDashboard.putNumber("Angle Offset", angleOffset);

        SmartDashboard.putString("Target Element", robotContainer.getTargetElement().toString());
        SmartDashboard.putString("Current Element", robotContainer.getCurrentElement().toString());

        robotContainer.updatePositionLevel(
                RobotContainer.firstController.getLeftBumperPressed(),
                RobotContainer.firstController.getRightBumperPressed());

        updatePID(
                p.getDouble(WristConstants.kP),
                i.getDouble(WristConstants.kI),
                d.getDouble(WristConstants.kD));
    }

    public void coneOuttakeAngled() {
    }
}

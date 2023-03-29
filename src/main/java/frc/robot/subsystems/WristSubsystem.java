// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
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
    public WPI_TalonFX wristMotorFirst, wristMotorSecond;
    private PIDController wristPID;
    // private ProfiledPIDController wristPID;

    private DutyCycleEncoder encoder;
    private double angleOffset = 0;

    @SuppressWarnings("unused")
    private int counter = 0; // use if absolute encoder still fluctuates too much
    @SuppressWarnings("unused")
    private LinearFilter encoderFilter; // TODO: use only if encoder values are still not stable enough

    private RobotContainer robotContainer;

    // ====================================================================
    // PID EDITING
    // ====================================================================

    ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");

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
    // CONSTRUCTOR
    // ====================================================================

    public WristSubsystem(RobotContainer robotContainer) {
        encoder = new DutyCycleEncoder(WristConstants.kEncoderID);
        encoderFilter = LinearFilter.movingAverage(5); // takes moving average over last 5 samples

        this.robotContainer = robotContainer;

        // ====================================================================
        // PID
        // ====================================================================

        // reg PID
        wristPID = new PIDController(
                WristConstants.kP,
                WristConstants.kI,
                WristConstants.kD);

        // Profiled PID
        // wristPID = new ProfiledPIDController(
        // WristConstants.kP,
        // WristConstants.kI,
        // WristConstants.kD,
        // new TrapezoidProfile.Constraints(
        // WristConstants.maxVelocity,
        // WristConstants.maxAcceleration));

        // ====================================================================
        // MOTOR SETUP
        // ====================================================================

        wristMotorFirst = new WPI_TalonFX(Constants.WristConstants.kWristIDLeft);
        wristMotorSecond = new WPI_TalonFX(Constants.WristConstants.kWristIDRight);

        wristMotorFirst.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        wristMotorFirst.setSelectedSensorPosition(0);
        wristMotorFirst.setInverted(true);
        wristMotorFirst.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 20, 0));
        wristMotorFirst.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 20, 0));

        wristMotorSecond.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        wristMotorSecond.setSelectedSensorPosition(0);
        wristMotorSecond.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 20, 0));
        wristMotorSecond.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 20, 0));

        // ====================================================================
    }

    // ====================================================================
    // WRIST MOVEMENT
    // ====================================================================

    public void setWrist(double percentOutput) {
        // Uses motor values
        // if (getAngleDegreesMotor() <= WristConstants.kWristSafePosition) {
        // percentOutput = Math.max(percentOutput, 0);
        // } else if (getAngleDegreesMotor() >= 90) {
        // percentOutput = Math.min(percentOutput, 0);
        // }

        // Uses abs encoder values
        if (getAngleDegreesAbs() <= WristConstants.kWristSafePosition) {
            percentOutput = Math.max(percentOutput, 0);
        } else if (getAngleDegreesAbs() >= 90) {
            percentOutput = Math.min(percentOutput, 0);
        }

        wristMotorFirst.set(ControlMode.PercentOutput, percentOutput);
        wristMotorSecond.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setPosition(double position) {
        // Uses motor for measurement
        double output = MathUtil.clamp(wristPID.calculate(getAngleDegreesMotor(),
                position), -0.6, 0.6);

        // double output = MathUtil.clamp(wristPID.calculate(getAngleDegreesAbs(),
        // position), -0.4, 0.4);

        // Uses abs encoder for measurement
        // double output = wristPID.calculate(getAngleDegreesAbs(), position);

        setWrist(output);
    }

    // ====================================================================
    // PID
    // ====================================================================

    public void updatePID(double kP, double kI, double kD) {
        wristPID.setP(kP);
        wristPID.setI(kI);
        wristPID.setD(kD);
    }

    // ====================================================================
    // GETTER METHODS
    // ====================================================================

    public double getPidError() {
        return wristPID.getPositionError();
    }

    public double getEncoder() {
        return encoderFilter.calculate((double) Math.round(encoder.getAbsolutePosition() * 1000d) / 1000d); // copied
                                                                                                            // from
                                                                                                            // stack
                                                                                                            // overflow
                                                                                                            // -
        // rounds to 3 decimal places
        // return encoder.getAbsolutePosition();
    }

    public double getPosition() {
        return (wristMotorFirst.getSelectedSensorPosition() + wristMotorSecond.getSelectedSensorPosition()) / 2;
    }

    public double getDriveRate() {
        return wristMotorFirst.getSelectedSensorVelocity();
    }

    public double getAngleDegreesMotor() {
        return WristConstants.convertTicksToRadians(wristMotorFirst.getSelectedSensorPosition(), angleOffset);
    }

    public double getAngleDegreesAbs() {
        return (getEncoder() - WristConstants.kEncoderOffset) * 360;
    }

    // ====================================================================

    @Override
    public void periodic() {
        if (counter <= 50) {
            counter++;
        }

        if (counter % 50 == 0) { // updates the encoder offset after a second of being enabled
            angleOffset = (getEncoder() - WristConstants.kEncoderOffset) * 360;
            counter++;
        }

        if (RobotContainer.slideController.getRightStickButtonPressed()) {
            angleOffset = (getEncoder() - WristConstants.kEncoderOffset) * 360;
            wristMotorFirst.setSelectedSensorPosition(0);
            wristMotorSecond.setSelectedSensorPosition(0);
        }

        SmartDashboard.putNumber("Wrist Stator", wristMotorFirst.getStatorCurrent());

        // UNCOMMENT ABOVE IF ENCODER STILL FLUCTUATES TOO MUCH

        SmartDashboard.putNumber("Wrist Setpoint", wristPID.getSetpoint()); // reg PID

        // Profiled PID
        // SmartDashboard.putNumber("Wrist Setpoint", wristPID.getSetpoint().position);

        SmartDashboard.putNumber("Position Level", robotContainer.getPositionLevel());
        SmartDashboard.putNumber("Wrist Encoder", getEncoder());
        SmartDashboard.putNumber("Wrist Angle Motor", getAngleDegreesMotor());
        SmartDashboard.putNumber("Wrist Angle Encoder", getAngleDegreesAbs());
        SmartDashboard.putNumber("Wrist Error", getPidError());
        SmartDashboard.putNumber("Angle Offset", angleOffset);

        SmartDashboard.putString("Target Element", robotContainer.getTargetElement().toString());
        SmartDashboard.putString("Current Element", robotContainer.getCurrentElement().toString());

        robotContainer.updatePositionLevel( // update position level based on bumpers
                RobotContainer.slideController.getLeftBumperPressed(),
                RobotContainer.slideController.getRightBumperPressed());

        updatePID(
                p.getDouble(WristConstants.kP),
                i.getDouble(WristConstants.kI),
                d.getDouble(WristConstants.kD));
    }
}

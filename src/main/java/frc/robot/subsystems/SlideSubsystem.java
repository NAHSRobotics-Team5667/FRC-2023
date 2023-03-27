// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SlideConstants;

public class SlideSubsystem extends SubsystemBase {
    private WPI_TalonFX rightSlide, leftSlide;
    private DigitalInput bottomLimitSwitch;
    private DigitalInput topLimitSwitch;
    public PIDController controller;

    // public ProfiledPIDController controller;

    // ====================================================================
    // PID EDITING
    // ====================================================================

    ShuffleboardTab slideTab = Shuffleboard.getTab("Slide");

    GenericEntry p = slideTab.add("Slide P", SlideConstants.kP)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", 0, "max", 10))
            .withPosition(0, 0)
            .getEntry();

    GenericEntry i = slideTab.add("Slide I", SlideConstants.kI)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", 0, "max", 10))
            .withPosition(1, 0)
            .getEntry();

    GenericEntry d = slideTab.add("Slide D", SlideConstants.kD)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", 0, "max", 10))
            .withPosition(2, 0)
            .getEntry();

    // ====================================================================

    /** Creates a new SlideSubsystem. */
    public SlideSubsystem() {

        // ====================================================================
        // Left Slide Motor
        // ====================================================================

        leftSlide = new WPI_TalonFX(Constants.SlideConstants.kLSlideID);
        leftSlide.setNeutralMode(NeutralMode.Brake);
        leftSlide.setSelectedSensorPosition(0);
        leftSlide.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        // Current limits
        // leftSlide.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
        // 40, 175, 0.75));
        // leftSlide.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
        // 40, 175, 0.75));

        // ====================================================================
        // Right Slide Motor
        // ====================================================================

        rightSlide = new WPI_TalonFX(Constants.SlideConstants.kRSlideID);
        rightSlide.setNeutralMode(NeutralMode.Brake);
        rightSlide.setSelectedSensorPosition(0);

        // Right slide current limits
        // rightSlide.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
        // 40, 175, 0.75));
        // rightSlide.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
        // 40, 175, 0.75));

        rightSlide.setInverted(true);

        // ====================================================================
        // Limit Switches
        // ====================================================================

        bottomLimitSwitch = new DigitalInput(Constants.SlideConstants.kBottomLimitSwitchId);
        topLimitSwitch = new DigitalInput(Constants.SlideConstants.kTopLimitSwitchId);

        // ====================================================================
        // PID
        // ====================================================================

        // PID Controller
        controller = new PIDController(
                SlideConstants.kP, SlideConstants.kI, SlideConstants.kD);

        // Profiled PID Controller
        // controller = new ProfiledPIDController(
        // SlideConstants.kP,
        // SlideConstants.kI,
        // SlideConstants.kD,
        // new TrapezoidProfile.Constraints(
        // SlideConstants.maxVelocity,
        // SlideConstants.maxAcceleration));

        // ====================================================================
    }

    // ====================================================================
    // ENCODERS
    // ====================================================================

    public double getRightRawEncoder() {
        return rightSlide.getSelectedSensorPosition();
    }

    public double getRightRawSpeed() {
        return Math.abs(getRightRawVelocity());
    }

    public double getRightRawVelocity() {
        return rightSlide.getSelectedSensorVelocity();
    }

    public void resetSlideEncoders(double rawUnits) {
        rightSlide.setSelectedSensorPosition(rawUnits);
        leftSlide.setSelectedSensorPosition(rawUnits);
    }

    // ====================================================================
    // LIMIT SWITCHES
    // ====================================================================

    public boolean getBottomLimitSwitch() {
        if (!bottomLimitSwitch.get()) {
            resetSlideEncoders(0);
        }
        return !bottomLimitSwitch.get();
    }

    public boolean getTopLimitSwitch() {
        return !topLimitSwitch.get(); // TODO: check that this does, in fact, return true when limit switch is pressed
    }

    // ====================================================================
    // UNIT CONVERSIONS
    // ====================================================================

    public double getSlideHeightInches() {
        return SlideConstants.rawUnitsToInches(getRightRawEncoder());
    }

    // ====================================================================
    // SLIDE MOVEMENT
    // ====================================================================

    public void setSlide(double percentOutput) {
        if (getBottomLimitSwitch()) {
            percentOutput = Math.max(percentOutput, 0);
        } else if (getTopLimitSwitch() || rightSlide.getSelectedSensorPosition() >= SlideConstants.maxEncoderTicks) {
            percentOutput = Math.min(percentOutput, 0);
        }

        leftSlide.set(ControlMode.PercentOutput, percentOutput);
        rightSlide.set(ControlMode.PercentOutput, percentOutput);
    }

    // ====================================================================
    // PID
    // ====================================================================

    public void updatePID(double kP, double kI, double kD) {
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public double getPIDOutput(double inchesSetpoint) {
        double output = MathUtil.clamp(
                controller.calculate(getSlideHeightInches(),
                        inchesSetpoint),
                -0.4, 0.4);

        // double output = controller.calculate(
        // getSlideHeightInches(),
        // inchesSetpoint);

        return output;
    }

    public void setSlidePIDInches(double inchesSetpoint) {
        double output = getPIDOutput(inchesSetpoint);

        setSlide(output);
    }

    public double getPositionError() {
        return controller.getPositionError();
    }

    // ====================================================================

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Slide Top Limit Switch", getTopLimitSwitch());
        SmartDashboard.putBoolean("Slide Bottom Limit Switch", getBottomLimitSwitch());
        SmartDashboard.putNumber("Right Slide Encoder", getRightRawEncoder());
        SmartDashboard.putNumber("Right Slide Inches", getSlideHeightInches());
        SmartDashboard.putNumber("Slide Error", getPositionError());

        SmartDashboard.putNumber("Slide Stator", rightSlide.getStatorCurrent());
        // SmartDashboard.putNumber("Slide Setpoint",
        // controller.getSetpoint().position); // Profiled PID Controller
        SmartDashboard.putNumber("Slide Setpoint", controller.getSetpoint()); // PID
        // Controller

        updatePID(
                p.getDouble(SlideConstants.kP),
                i.getDouble(SlideConstants.kI),
                d.getDouble(SlideConstants.kD));

        // Update drivetrain speed depending on height of slide - prevents tipping
        // if (getSlideHeightInches() < 15) {
        // robotContainer.setSpeedMultiplier(0.7);
        // robotContainer.setTurnMultiplier(0.7);

        // } else if (getSlideHeightInches() >= 15 && getSlideHeightInches() < 30) {
        // robotContainer.setSpeedMultiplier(0.7);
        // robotContainer.setTurnMultiplier(0.5);

        // } else if (getSlideHeightInches() >= 30 && getSlideHeightInches() < 45) {
        // robotContainer.setSpeedMultiplier(0.5);
        // robotContainer.setSpeedMultiplier(0.4);

        // } else { // getSlideHeightInches() >= 45
        // robotContainer.setSpeedMultiplier(0.3);
        // robotContainer.setSpeedMultiplier(0.3);

        // }
    }
}

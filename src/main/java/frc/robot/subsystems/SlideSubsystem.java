// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SlideConstants;

public class SlideSubsystem extends SubsystemBase {
    private WPI_TalonFX rightSlide, leftSlide;
    private DigitalInput bottomLimitSwitch;
    private DigitalInput topLimitSwitch;
    public PIDController controller = new PIDController(.1, 0, 0);

    ShuffleboardTab slideTab = Shuffleboard.getTab("Slide");

    // ====================================================================
    // PID EDITING
    // ====================================================================
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
        leftSlide = new WPI_TalonFX(Constants.SlideConstants.kLSlideID);
        leftSlide.setNeutralMode(NeutralMode.Brake);
        leftSlide.setSelectedSensorPosition(0);
        leftSlide.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 175, 0.75));
        leftSlide.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 175, 0.75));

        rightSlide = new WPI_TalonFX(Constants.SlideConstants.kRSlideID);
        rightSlide.setNeutralMode(NeutralMode.Brake);
        rightSlide.setSelectedSensorPosition(0);
        leftSlide.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 175, 0.75));
        leftSlide.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 175, 0.75));

        rightSlide.setInverted(true);

        leftSlide.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        bottomLimitSwitch = new DigitalInput(Constants.SlideConstants.kBottomLimitSwitchId);
        topLimitSwitch = new DigitalInput(Constants.SlideConstants.kTopLimitSwitchId);

        // controller.setTolerance(0);
    }

    public double getRightRawEncoder() {
        return rightSlide.getSelectedSensorPosition();
    }

    public void resetSlideEncoders() {
        rightSlide.setSelectedSensorPosition(0);
        leftSlide.setSelectedSensorPosition(0);
    }

    public boolean getBottomLimitSwitch() {
        if (!bottomLimitSwitch.get()) {
            resetSlideEncoders();
        }
        return !bottomLimitSwitch.get();
    }

    public boolean getTopLimitSwitch() {
        // if (!topLimitSwitch.get()) {
        // leftSlide.setSelectedSensorPosition(277000);
        // rightSlide.setSelectedSensorPosition(277000);
        // }
        return !topLimitSwitch.get();
    }

    public double getDriveRate() {
        return leftSlide.getSelectedSensorVelocity();
    }

    public void setSlide(double percentOutput) {
        if (getBottomLimitSwitch()) {
            percentOutput = Math.max(percentOutput, 0);
        } else if (getTopLimitSwitch() || rightSlide.getSelectedSensorPosition() >= SlideConstants.maxEncoderTicks) {
            percentOutput = Math.min(percentOutput, 0);
        }

        leftSlide.set(ControlMode.PercentOutput, percentOutput);
        rightSlide.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setSlidePIDEncoder(double encoderSetpoint) {
        double output = controller.calculate(getRightRawEncoder(), encoderSetpoint);
        setSlide(output);
    }

    public void setSlidePIDInches(double inchesSetpoint) {
        double output = MathUtil.clamp(
                controller.calculate(SlideConstants.rawUnitsToInches(getRightRawEncoder()), inchesSetpoint), -0.3, 0.3);
        setSlide(output);
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public void setPosition(double inchesSetpoint) {
        setSlidePIDInches(inchesSetpoint);
    }

    public double getVelocity() {
        return leftSlide.getSelectedSensorVelocity();
    }

    public double getPositionError() {
        return controller.getPositionError();
    }

    public double getSlideOutput() {
        return 0; // TODO
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Slide Top Limit Switch", getTopLimitSwitch());
        SmartDashboard.putBoolean("Slide Bottom Limit Switch", getBottomLimitSwitch());
        SmartDashboard.putNumber("Right Slide Encoder", getRightRawEncoder());
        SmartDashboard.putNumber("Right Slide Inches", SlideConstants.rawUnitsToInches(getRightRawEncoder()));
        SmartDashboard.putNumber("Slide Error", getPositionError());

        SmartDashboard.putNumber("Slide Stator", rightSlide.getStatorCurrent());
        SmartDashboard.putNumber("Slide Setpoint", controller.getSetpoint());

        updatePID(
                p.getDouble(SlideConstants.kP),
                i.getDouble(SlideConstants.kI),
                d.getDouble(SlideConstants.kD));
    }
}

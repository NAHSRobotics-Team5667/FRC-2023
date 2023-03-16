// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SlideConstants;

public class SlideSubsystem extends SubsystemBase {
    private WPI_TalonFX m_rightSlide, m_leftSlide;
    private DigitalInput m_bottomLimitSwitch;
    
    @SuppressWarnings("unused")
    private WristSubsystem m_wrist;
    // private DutyCycleEncoder absEncoderHeight = new
    // DutyCycleEncoder(Constants.SlideConstants.EncoderId);
    // private double trueHeightOffset = absEncoderHeight.getAbsolutePosition() -
    // Constants.SlideConstants.EncoderOffset;
    @SuppressWarnings("unused")
    private SimpleMotorFeedforward m_slideFeedForward = new SimpleMotorFeedforward(0, 0, 0);

    private PIDController controller = new PIDController(.25, 0, 0);

    @SuppressWarnings("unused")
    private ProfiledPIDController slideController = new ProfiledPIDController(0.5, 0, 0,
            new TrapezoidProfile.Constraints(
                    15, 5));

    // maybe add a feed forward? May be unnecessary though
    /** Creates a new SlideSubsystem. */
    public SlideSubsystem() {
        m_leftSlide = new WPI_TalonFX(Constants.SlideConstants.kLSlideID);
        m_leftSlide.setNeutralMode(NeutralMode.Brake);
        m_leftSlide.setSelectedSensorPosition(0);

        m_rightSlide = new WPI_TalonFX(Constants.SlideConstants.kRSlideID);
        m_rightSlide.setNeutralMode(NeutralMode.Brake);
        m_rightSlide.setSelectedSensorPosition(0);

        m_rightSlide.setInverted(true);

        m_leftSlide.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        m_bottomLimitSwitch = new DigitalInput(Constants.SlideConstants.kLimitSwitchId);
    }

    public double getRightRawEncoder() {
        return m_rightSlide.getSelectedSensorPosition();
    }

    public boolean getLimitSwitch() {
        if (!m_bottomLimitSwitch.get()) {
            m_leftSlide.setSelectedSensorPosition(0);
            m_rightSlide.setSelectedSensorPosition(0);
        }

        return !m_bottomLimitSwitch.get();
    }

    public double getDriveRate() {
        return m_leftSlide.getSelectedSensorVelocity();
    }

    // public void moveSlide(double desiredState){
    // double currentLeftPosition = getLeftPosition();
    // double outputLeft = controllerLeft.calculate(currentLeftPosition,
    // desiredState);
    // double slideFeedForward = m_slideFeedForward.calculate(getDriveRate());
    // m_leftSlide.setVoltage(outputLeft + slideFeedForward);
    // double currentRightPosition = getRightPosition();
    // double outputRight = controllerLeft.calculate(currentRightPosition,
    // desiredState);
    // m_leftSlide.setVoltage(outputRight + slideFeedForward);

    // }

    public void setSlide(double percentOutput) {
        if (getLimitSwitch()) {
            percentOutput = Math.max(percentOutput, 0);
        } else {
            if (getRightRawEncoder() >= 250000) {
                percentOutput = Math.min(percentOutput, 0);
            }
        }

        m_leftSlide.set(ControlMode.PercentOutput, percentOutput);
        m_rightSlide.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setSlidePIDEncoder(double encoderSetpoint) {
        double output = controller.calculate(getRightRawEncoder(), encoderSetpoint);
        setSlide(output);
    }

    public void setSlidePIDInches(double inchesSetpoint) {
        // double output =
        // slideController.calculate(SlideConstants.rawUnitsToInches(getRightRawEncoder()),
        // inchesSetpoint);
        double output = MathUtil.clamp(
                controller.calculate(SlideConstants.rawUnitsToInches(getRightRawEncoder()), inchesSetpoint), -0.9, 0.9);
        setSlide(output);
    }

    public void setPosition(double inchesSetpoint) {
        setSlidePIDInches(inchesSetpoint);
    }

    public double getVelocity() {
        return m_leftSlide.getSelectedSensorVelocity();
    }

    public void setPosition(int bumperPos) {
    }

    // public double getLeftPosition(){
    // return (m_leftSlide.getSelectedSensorPosition() *
    // Constants.SlideConstants.kSlideConstant) - trueHeightOffset;

    // }
    // public double getRightPosition(){
    // return (m_rightSlide.getSelectedSensorPosition() *
    // Constants.SlideConstants.kSlideConstant) - trueHeightOffset;

    // }
    // public boolean isLeftAndRightBalanced(){
    // return (Math.abs(getLeftPosition()-getRightPosition()) < .01);
    // }

    public double getSlideOutput() {
        return 0; // TODO
    }

    // make a function that gets the number of ticks
    // make a functon that set motor to number of ticks
    // make a function that designates levels for game
    // try to optimize
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Slide Limit Switch", getLimitSwitch());
        SmartDashboard.putNumber("Right Slide Encoder", getRightRawEncoder());
        SmartDashboard.putNumber("Right Slide Inches", SlideConstants.rawUnitsToInches(getRightRawEncoder()));
        // SmartDashboard.putBoolean("Balance", isLeftAndRightBalanced());
        // This method will be called once per scheduler run
    }
}

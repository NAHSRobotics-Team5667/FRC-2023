// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.SwerveModulePosition;

public class SwerveModule {
    private static final double kWheelRadius = 0.0508;
    private static final double kEncoderResolution = 2048;

    private static final double kDriveGearRatio = 6.54;
    private static final double kTurnGearRatio = 15.43;

    private static final double kDriveEncoderConstant = (2 * kWheelRadius * Math.PI)
            / (kEncoderResolution * kDriveGearRatio);
    private static final double kTurnEncoderConstant = 2 * Math.PI / (kTurnGearRatio * kEncoderResolution);

    private static final double kModuleMaxAngularVelocity = DrivetrainSubsystem.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_turningMotor;

    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            2,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0);

    private double angleOffset = 0;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel      PWM output for the drive motor.
     * @param turningMotorChannel    PWM output for the turning motor.
     * @param driveEncoderChannelA   DIO input for the drive encoder channel A
     * @param driveEncoderChannelB   DIO input for the drive encoder channel B
     * @param turningEncoderChannelA DIO input for the turning encoder channel A
     * @param turningEncoderChannelB DIO input for the turning encoder channel B
     */
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            double angleOffset) {
        m_driveMotor = new WPI_TalonFX(driveMotorChannel);
        m_turningMotor = new WPI_TalonFX(turningMotorChannel);
        this.angleOffset = angleOffset;

        m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        m_turningMotor.setSelectedSensorPosition(0);
        m_driveMotor.setSelectedSensorPosition(0);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveEncoderDistance(), new Rotation2d(getTurnEncoderDistance()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDriveEncoderDistance(), new Rotation2d(getTurnEncoderDistance()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(getTurnEncoderDistance()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(getDriveEncoderRate(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(getTurnEncoderDistance(),
                state.angle.getRadians());

        final double turnFeedforward = 0; // m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    }

    public void driveVoltage(double voltage) {
        m_driveMotor.setVoltage(voltage);
        m_turningMotor.setVoltage(voltage);
    }

    /**
     * Get the drive motor encoder's rate
     * 
     * @return the drive motor encoder's rate
     */
    public double getDriveEncoderRate() {
        return m_driveMotor.getSelectedSensorVelocity() * kDriveEncoderConstant * 10;
    }

    /**
     * Get the drive motor encoder's distance
     * 
     * @return the drive motor encoder's distance
     */
    public double getDriveEncoderDistance() {
        return m_driveMotor.getSelectedSensorPosition() * kDriveEncoderConstant;
    }

    /**
     * Get the turn motor angle
     * 
     * @return get the turn motor angle
     */
    public double getTurnEncoderDistance() {
        return (m_turningMotor.getSelectedSensorPosition() * kTurnEncoderConstant) - this.angleOffset;
    }

    public double getAngleSetpoint() {
        return m_turningPIDController.getSetpoint().position;
    }

    public double getDriveSetpoint() {
        return m_drivePIDController.getSetpoint();
    }

    public double getTurnEncoderRaw() {
        return m_turningMotor.getSelectedSensorPosition();
    }

    public boolean turnAtSetpoint() {
        return Math.abs(m_turningPIDController.getSetpoint().position - getTurnEncoderDistance()) < 0.1;
    }
}
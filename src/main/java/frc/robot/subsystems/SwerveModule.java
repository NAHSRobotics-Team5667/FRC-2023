// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    public static final double kDriveEncoderConstant = (2 * DriveConstants.kWheelRadius * Math.PI)
            / (DriveConstants.kEncoderResolution * DriveConstants.kDriveGearRatio);

    @SuppressWarnings("unused")
    private static final double kModuleMaxAngularVelocity = DriveConstants.kMaxAngularSpeed,
            kModuleMaxAngularAcceleration = DriveConstants.kMaxAngularAcceleration; // radians per second squared

    private final WPI_TalonFX driveMotor, turningMotor;
    private final PIDController drivePIDController = new PIDController(.031576, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    // public final ProfiledPIDController turningPIDController;
    public final PIDController turningPIDController;

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(.096682, 2.2041, .54385);
    double trueEncoderOffset = 100, trueEncoderOffsetTest = 0;
    double[] averageOffsetBoi;
    @SuppressWarnings("unused")
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.26284, 0.27578, 0.0038398);
    // TODO: do something idk
    DutyCycleEncoder Encoder;
    double angleOffset = 0;

    @SuppressWarnings("unused")
    private final LinearFilter filter = LinearFilter.movingAverage(10000); // average over last 5 samples

    private double sampleCounter = 0, encoderSampleSum = 0;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel   PWM output for the drive motor.
     * @param turningMotorChannel PWM output for the turning motor.
     * @param angleOffset         Offset for the turning encoder
     * @param Encoder             Encoder for the turning motor
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel, double angleOffset, DutyCycleEncoder Encoder) {
        this(
                driveMotorChannel,
                turningMotorChannel,
                angleOffset,
                Encoder,
                DriveConstants.kTurnKp,
                DriveConstants.kTurnKi,
                DriveConstants.kTurnKd);
    }

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
     * @param turnKp                 Turn P value
     * @param turnKi                 Turn I value
     * @param turnKd                 Turn D value
     */
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            double angleOffset,
            DutyCycleEncoder Encoder,
            double turnKp,
            double turnKi,
            double turnKd) {

        this.Encoder = Encoder;
        this.angleOffset = angleOffset;
        if (trueEncoderOffset > 1) {
            trueEncoderOffset = this.Encoder.getAbsolutePosition() - angleOffset;
        }

        // turningPIDController = new ProfiledPIDController(turnKp, turnKi, turnKd,
        // new TrapezoidProfile.Constraints(
        // kModuleMaxAngularVelocity,
        // kModuleMaxAngularAcceleration));

        this.turningPIDController = new PIDController(turnKp, turnKi, turnKd);

        this.driveMotor = new WPI_TalonFX(driveMotorChannel);
        this.turningMotor = new WPI_TalonFX(turningMotorChannel);
        this.turningMotor.setNeutralMode(NeutralMode.Brake);
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.turningPIDController.setTolerance(0, 0);
        this.turningMotor.setSelectedSensorPosition(0);
        this.driveMotor.setSelectedSensorPosition(0);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
    public edu.wpi.first.math.kinematics.SwerveModulePosition getPosition() {
        return new edu.wpi.first.math.kinematics.SwerveModulePosition(
                getDriveEncoderDistance(), new Rotation2d(getTurnEncoderDistance()));
    }
    // private double angleOffset = this.angleOffset;

    // public ProfiledPIDController getTurningPID() {
    // return this.m_turningPIDController;
    // }

    public PIDController getTurningPID() {
        return this.turningPIDController;
    }

    public PIDController getDrivePID() {
        return this.drivePIDController;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        // SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        // new Rotation2d(getTurnEncoderDistance()));

        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(getTurnEncoderDistance()));
        // Calculate the drive output from the drive PID controller.

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(getTurnEncoderDistance(),
                state.angle.getRadians());

        final double turnFeedforward = 0; // turnFeedforward.calculate(state.angle.getRadians());

        turningMotor.setVoltage(turnOutput + turnFeedforward);
        final double driveOutput = drivePIDController.calculate(getDriveEncoderRate(), state.speedMetersPerSecond);
        // double driveOutput = 0;
        driveMotor.setVoltage(driveOutput + driveFeedforward);
        turningPIDController.getPositionError();
    }

    public void updateTurnPID(double kP, double kI, double kD) {
        turningPIDController.setP(kP);
        turningPIDController.setI(kI);
        turningPIDController.setD(kD);
    }

    public void driveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
        turningMotor.setVoltage(voltage);
    }

    /**
     * Get the drive motor encoder's rate
     * 
     * @return the drive motor encoder's rate
     */
    public double getDriveEncoderRate() {
        return driveMotor.getSelectedSensorVelocity() * kDriveEncoderConstant * 10;
    }

    public double getDriveEncoderRaw() {
        return driveMotor.getSelectedSensorPosition();
    }

    /**
     * Get the drive motor encoder's distance
     * 
     * @return the drive motor encoder's distance
     */
    public double getDriveEncoderDistance() {
        return driveMotor.getSelectedSensorPosition() * kDriveEncoderConstant;
    }

    /**
     * @return get the turn motor angle
     */
    public double getTurnEncoderDistance() {
        trueEncoderOffsetTest = this.Encoder.getAbsolutePosition() - angleOffset;

        // return ((this.Encoder.getDistance()- angleOffset)*2*Math.PI);
        return (turningMotor.getSelectedSensorPosition() * DriveConstants.kTurnEncoderConstant)
                - (trueEncoderOffset * 2 * Math.PI);
    }

    public double getAbsTurnEncoder() {
        // return filter.calculate(this.Encoder.getAbsolutePosition() - angleOffset);
        return this.Encoder.getAbsolutePosition() /*- angleOffset*/;
    }

    public double getAbsTurnEncoderRadians() {
        return getAbsTurnEncoder() * 2 * Math.PI;
    }

    // public double getAngleSetpoint() {
    // return turningPIDController.getSetpoint().position;
    // }

    public double getAngleSetpoint() {
        return turningPIDController.getSetpoint();
    }

    public double getDriveSetpoint() {
        return drivePIDController.getSetpoint();
    }

    public double getTurnEncoderRaw() {
        return turningMotor.getSelectedSensorPosition();
    }

    // public boolean turnAtSetpoint() {
    // return Math.abs(m_turningPIDController.getSetpoint().position -
    // getTurnEncoderDistance()) < 0.1;
    // }

    public boolean turnAtSetpoint() {
        return Math.abs(turningPIDController.getSetpoint() - getTurnEncoderDistance()) < 0.1;
    }

    public double collectEncoderSample() {
        if (sampleCounter < 50.0) {
            encoderSampleSum += getAbsTurnEncoder();
            sampleCounter++;
        } else if (sampleCounter == 50) {
            trueEncoderOffset = encoderSampleSum / sampleCounter;
        }
        return trueEncoderOffset;
    }

    public double getOffset() {
        return trueEncoderOffset;
    }
}
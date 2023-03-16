// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer.GamePiece;

public class WristSubsystem extends SubsystemBase {

    public int bumperPos = 0;
    public WPI_TalonFX m_wristMotorFirst, m_wristMotorSecond;
    private PIDController wristPID = new PIDController(.02, 0, 0);
    private SimpleMotorFeedforward m_wristFeedForward = new SimpleMotorFeedforward(0, 0, 0);

    private DutyCycleEncoder m_encoder;

    private double angleOffset = 0;

    private int counter = 0;

    private RobotContainer m_robotContainer;

    private double maxBumperPos = 0;

    /** Creates a new WristSubsystem. */
    public WristSubsystem(RobotContainer m_robotContainer) {
        m_encoder = new DutyCycleEncoder(5);

        wristPID.setTolerance(2.5);

        m_wristMotorFirst = new WPI_TalonFX(Constants.WristConstants.kWristIDLeft);
        m_wristMotorSecond = new WPI_TalonFX(Constants.WristConstants.kWristIDRight);

        m_wristMotorFirst.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        m_wristMotorFirst.setSelectedSensorPosition(0);

        m_wristMotorSecond.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        m_wristMotorSecond.setSelectedSensorPosition(0);

        m_wristMotorFirst.setInverted(true);

        this.m_robotContainer = m_robotContainer;

        // angleOffset = (getEncoder() - WristConstants.kEncoderOffset) * 360;
    }

    public void setWrist(double percentOutput) {
        if (getAngleDegrees() < -125) {
            percentOutput = Math.max(percentOutput, 0);
        } else if (getAngleDegrees() > 90) {
            percentOutput = Math.min(percentOutput, 0);
        }

        m_wristMotorFirst.set(ControlMode.PercentOutput, percentOutput);
        m_wristMotorSecond.set(ControlMode.PercentOutput, percentOutput);
    }

    public double getEncoder() {
        return m_encoder.getAbsolutePosition();
    }

    public void setPosition(double position) {
        double output = MathUtil.clamp(wristPID.calculate(getAngleDegrees(), position), -0.4, 0.4);
        setWrist(output);
    }

    public int getBumperPos() {
        return bumperPos;
    }

    public void setBumperPos(int bumperPos) {
        this.bumperPos = bumperPos;
    }

    public double getPosition() {
        return (m_wristMotorFirst.getSelectedSensorPosition() + m_wristMotorSecond.getSelectedSensorPosition()) / 2;
    }

    public double getDriveRate() {
        return m_wristMotorFirst.getSelectedSensorVelocity();
    }

    public double pidError() {
        return wristPID.getPositionError();
    }

    // public void maintainSafePosition() {
    // double currentPosition = getPosition();
    // double outputWrist = wristPID.calculate(currentPosition,
    // Constants.WristConstants.kWristSafePosition);
    // double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
    // m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
    // m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);

    // }

    // make these dependent on bumperPos, array of setPoints
    public void coneIntakeAngled() { // TODO: this method doesnt do shit
        @SuppressWarnings("unused")
        double currentPosition = getPosition();
        // double outputWrist =

        // wristPID.calculate(currentPosition,
        // Constants.WristConstants.kWristConeIntakeSetpoint);
        // double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        // m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
        // m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);

        // Wrist Angled for Cone intake
    }

    // public void coneOuttakeAngled() {
    // double currentPosition = getPosition();
    // double outputWrist = wristPID.calculate(currentPosition,
    // Constants.WristConstants.kWristConeOuttakeSetpoint[bumperPos]);
    // double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
    // m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
    // m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);
    // }

    // public void cubeIntakeAngled() {
    // // double currentPosition = getPosition();
    // // double outputWrist = wristPID.calculate(currentPosition,
    // // Constants.WristConstants.kWristCubeIntakeSetpoint[bumperPos]);
    // // double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
    // // m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
    // // m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);
    // // Wrist Angled for Cube intake
    // }

    // public void cubeOuttakeAngled() {
    // double currentPosition = getPosition();
    // double outputWrist = wristPID.calculate(currentPosition,
    // WristConstants.kWristCubeOuttakeSetpoint[bumperPos]);
    // double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
    // m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
    // m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);
    // // Wrist Angled for Cube intake
    // }

    public double getAngleDegrees() {
        return WristConstants.convertTicksToRadians(m_wristMotorFirst.getSelectedSensorPosition(), angleOffset);
    }

    @Override
    public void periodic() {
        if (counter <= 50) {
            counter++;
        }

        if (counter == 50) {
            angleOffset = (getEncoder() - WristConstants.kEncoderOffset) * 360;
        }

        SmartDashboard.putNumber("Wrist Setpoint", wristPID.getSetpoint());

        // if (counter % 5 == 0 && counter >= 30) {
        // angleOffset = (getEncoder() - WristConstants.kEncoderOffset) * 360;
        // }

        // if (Math.abs((getEncoder() * 360) - getAngleDegrees()) > 1) {
        // angleOffset = (getEncoder() - WristConstants.kEncoderOffset) * 360;

        // m_wristMotorFirst.setSelectedSensorPosition(0);
        // m_wristMotorSecond.setSelectedSensorPosition(0);
        // }

        // placed here because i need it updated constantly and dont want to deal with
        // putting it in a command properly
        if (m_robotContainer.getTargetElement().equals(GamePiece.CONE)) {
            maxBumperPos = WristConstants.coneIntakeSetpoints.length;

        } else if (m_robotContainer.getTargetElement().equals(GamePiece.CUBE)) {
            maxBumperPos = WristConstants.cubeIntakeSetpoints.length;

        } else if (!m_robotContainer.getCurrentElement().equals(GamePiece.NONE)) {
            maxBumperPos = 3;
        }

        if (RobotContainer.m_controller.getRightBumperPressed()) {
            if (bumperPos < maxBumperPos) {
                bumperPos++;
            }
        }

        if (RobotContainer.m_controller.getLeftBumperPressed()) {
            if (bumperPos > 0) {
                bumperPos--;
            }
        }

        if (RobotContainer.m_controller.getLeftStickButtonPressed()) {
            bumperPos = 0;
        }

        SmartDashboard.putNumber("Bumper Pos", getBumperPos());
        SmartDashboard.putNumber("Wrist Encoder", getEncoder());
        SmartDashboard.putNumber("Wrist Angle", getAngleDegrees());
        SmartDashboard.putNumber("Angle Offset", angleOffset);

        SmartDashboard.putString("Target Element", m_robotContainer.getTargetElement().toString());
        SmartDashboard.putString("Current Element", m_robotContainer.getCurrentElement().toString());

        // if (RobotContainer.m_controller.getYButton()) {
        // bumperPos = 0;
        // }

        // if (RobotContainer.m_controller.getXButton()) {
        // bumperPos = 3;
        // }

        // This method will be called once per scheduler run
    }

    public void coneOuttakeAngled() {
    }
}

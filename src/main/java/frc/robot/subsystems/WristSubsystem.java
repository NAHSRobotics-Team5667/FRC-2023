// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

    public int bumperPos = 0;
    public WPI_TalonFX m_wristMotorFirst;
    public WPI_TalonFX m_wristMotorSecond;
    private PIDController wristPID = new PIDController(.5, 0, 0);
    private SimpleMotorFeedforward m_wristFeedForward = new SimpleMotorFeedforward(0, 0, 0);

    private DutyCycleEncoder m_encoder;
    private double angleOffset;

    private double angle;

    /** Creates a new WristSubsystem. */
    public WristSubsystem() {
        m_encoder = new DutyCycleEncoder(5);
        
        angleOffset = getEncoder() * 360;

        m_wristMotorFirst = new WPI_TalonFX(Constants.WristConstants.kWristIDLeft);
        m_wristMotorSecond = new WPI_TalonFX(Constants.WristConstants.kWristIDRight);

        m_wristMotorFirst.setInverted(true);

        m_wristMotorFirst.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        m_wristMotorFirst.setSelectedSensorPosition(0);

        m_wristMotorSecond.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        m_wristMotorSecond.setSelectedSensorPosition(0);

        angle = getEncoder() * 360;
    }

    public double getAngle() {
        return angle;
    }

    public void setWrist(double percentOutput) {
        // if (m_encoder.getAbsolutePosition() <= 0.1) {
        //     percentOutput = Math.max(percentOutput, 0);
        // } else if (m_encoder.getAbsolutePosition() >= (215 / 360)) {
        //     percentOutput = Math.min(percentOutput, 0);
        // }

        // if (m_encoder.getAbsolutePosition() <= 0.1) {
        //     percentOutput = Math.max(percentOutput, 0);
        // } else if (m_encoder.getAbsolutePosition() >= (215 / 360)) {
        //     percentOutput = Math.min(percentOutput, 0);
        // }


        m_wristMotorFirst.set(ControlMode.PercentOutput, percentOutput);
        m_wristMotorSecond.set(ControlMode.PercentOutput, percentOutput);
    }

    public double getEncoder() {
        return m_encoder.getAbsolutePosition();
    }

    public void setPosition(double position){
        if (getEncoder() > position){
            setWrist(-0.1);
        }else if(getEncoder() < position){
            setWrist(0.1);
        } else {
            setWrist(0);
        }
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

    public void maintainSafePosition() {
        double currentPosition = getPosition();
        double outputWrist = wristPID.calculate(currentPosition, Constants.WristConstants.kWristSafePostion);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
        m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);

    }

    public void coneIntakeAngled() {
        double currentPosition = getPosition();
        double outputWrist = wristPID.calculate(currentPosition, Constants.WristConstants.kWristConeSetpoint);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
        m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);

        // Wrist Angled for Cone intake
    }

    public void coneOuttakeAngled() {
        double currentPosition = getPosition();
        double outputWrist = wristPID.calculate(currentPosition, Constants.WristConstants.kWristConeOuttakeSetpoint[bumperPos]);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
        m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);
    }

    public void cubeAngled(double wristPoint) {
        double currentPosition = getPosition();
        double outputWrist = wristPID.calculate(currentPosition, wristPoint);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
        m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);
        // Wrist Angled for Cube intake
    }

    @Override
    public void periodic() {
        angle = WristConstants.convertTicksToDegrees(m_wristMotorFirst.getSelectedSensorPosition(), angleOffset);

        SmartDashboard.putNumber("Wrist Encoder", getEncoder());
        SmartDashboard.putNumber("Wrist Angle", angle);
        // placed here because i need it updated constantly and dont want to deal with
        // putting it in a command properly
        if (RobotContainer.m_controller.getLeftBumperPressed()) {
            if (bumperPos != 3) {
                bumperPos++;
            }
        }
        if (RobotContainer.m_controller.getRightBumperPressed()) {
            if (bumperPos != 0) {
                bumperPos = -1;
            }
        }
        if (RobotContainer.m_controller.getYButton()) {
            bumperPos = 0;
        }
        if (RobotContainer.m_controller.getXButton()) {
            bumperPos = 3;
        }

        // This method will be called once per scheduler run
    }
}

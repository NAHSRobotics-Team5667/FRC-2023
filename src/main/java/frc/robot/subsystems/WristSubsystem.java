// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

    public int bumperPos = 0;
    public WPI_TalonFX m_wristMotorFirst;
    public WPI_TalonFX m_wristMotorSecond;
    private PIDController wristPID = new PIDController(.5, 0, 0);
    private SimpleMotorFeedforward m_wristFeedForward = new SimpleMotorFeedforward(0, 0, 0);

    /** Creates a new WristSubsystem. */
    public WristSubsystem() {

        m_wristMotorFirst = new WPI_TalonFX(Constants.WristConstants.kWristIDLeft);
        m_wristMotorSecond = new WPI_TalonFX(Constants.WristConstants.kWristIDRight);

        m_wristMotorFirst.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        m_wristMotorFirst.setSelectedSensorPosition(0);

        m_wristMotorSecond.setNeutralMode(NeutralMode.Brake); // DO NOT CHANGE FROM BRAKE
        m_wristMotorSecond.setSelectedSensorPosition(0);
    }

    public void setWrist(double percentOutput) {
        m_wristMotorFirst.set(ControlMode.PercentOutput, percentOutput);
        m_wristMotorSecond.set(ControlMode.PercentOutput, -percentOutput);
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
    //make these dependent on bumperPos, array of setPoints
    public void coneIntakeAngled() {
        double currentPosition = getPosition();
        double outputWrist = wristPID.calculate(currentPosition, Constants.WristConstants.kWristConeIntakeSetpoint[bumperPos]);
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

    public void cubeIntakeAngled() {
        double currentPosition = getPosition();
        double outputWrist = wristPID.calculate(currentPosition, Constants.WristConstants.kWristCubeIntakeSetpoint[bumperPos]);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
        m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);
        // Wrist Angled for Cube intake
    }
    
    public void cubeOuttakeAngled() {
        double currentPosition = getPosition();
        double outputWrist = wristPID.calculate(currentPosition, WristConstants.kWristCubeOuttakeSetpoint[bumperPos]);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotorFirst.setVoltage(outputWrist + wristFeedForward);
        m_wristMotorSecond.setVoltage(outputWrist + wristFeedForward);
        // Wrist Angled for Cube intake
    }


    @Override
    public void periodic() {
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

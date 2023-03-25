// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.revrobotics.Rev2mDistanceSensor;
// import com.revrobotics.Rev2mDistanceSensor.Port;
// import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CurrentSpikeCounter;

public class IntakeSubsystem extends SubsystemBase {
    public WPI_TalonFX intake; // intake motor
    private CurrentSpikeCounter spike_counter = new CurrentSpikeCounter(Constants.SlideConstants.CurrentThreshold,
            Constants.SlideConstants.CurrentDeadband);

    // private Rev2mDistanceSensor m_distanceSensor;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        intake = new WPI_TalonFX(Constants.IntakeConstants.kIntakeID);
        intake.setNeutralMode(NeutralMode.Brake);

        // m_distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        // m_distanceSensor.setAutomaticMode(true);

        // claw.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,
        // 100, 0.5));
        // claw.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40,
        // 100, 0.5));
    }

    public boolean isPieceIntaken() {
        return spike_counter.update(intake.getStatorCurrent(), false);
    }

    // ===========================================================================
    // MOTOR
    // ===========================================================================

    /**
     * Purpose: To return current motor output voltage to use as de facto sensor
     * 
     * @return: Current motor voltage
     */
    public double getMotorOutputVoltage() {
        return intake.getMotorOutputVoltage();
    }

    public double getMotorSpeed() {
        return intake.getSelectedSensorVelocity();
    }

    public double getMotorInput() {
        return intake.getBusVoltage();
    }

    public double getPosition() {
        return intake.getSelectedSensorPosition();
    }

    /**
     * Purpose: spin intake wheels
     * 
     * @param percentOutput: Percent of output using Motor Controller
     */
    public void setIntake(double percentOutput) {
        this.intake.set(ControlMode.PercentOutput, percentOutput);
    }

    // ===========================================================================
    // DISTANCE SENSOR
    // ===========================================================================

    // public boolean isRangeValid() {
    // return m_distanceSensor.isRangeValid();
    // }

    // public double getRangeInches() {
    // return m_distanceSensor.getRange(Unit.kInches);
    // }

    // ===========================================================================

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClawVoltage", intake.getMotorOutputVoltage());
        SmartDashboard.putNumber("ClawTempurature", intake.getTemperature());
        SmartDashboard.putNumber("ClawCurrent", intake.getStatorCurrent());
        isPieceIntaken();

        // SmartDashboard.putBoolean("Distance Range Valid",
        // m_distanceSensor.isRangeValid());
        // SmartDashboard.putNumber("Distance Sensor Range",
        // m_distanceSensor.getRange(Unit.kInches));
    }
}

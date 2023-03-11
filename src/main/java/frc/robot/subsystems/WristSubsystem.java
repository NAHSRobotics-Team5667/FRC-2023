// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private WPI_TalonFX m_wrist;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {

    m_wrist = new WPI_TalonFX(Constants.WristConstants.kWristID);
    m_wrist.setNeutralMode(NeutralMode.Brake); //DO NOT CHANGE FROM BRAKE
    m_wrist.setSelectedSensorPosition(0);
  }

  public void setWrist(double percentOutput) {
    m_wrist.set(ControlMode.PercentOutput, percentOutput);
  }

  public void coneAngled(){
  //Wrist Angled for Cone intake
  }
  public void cubeAngled(){
  //Wrist Angled for Cube intake
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

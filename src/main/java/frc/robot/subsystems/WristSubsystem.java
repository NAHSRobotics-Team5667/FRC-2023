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

public class WristSubsystem extends SubsystemBase {
  public int bumperPos = 0;
  public WPI_TalonFX m_wristMotor;
  private PIDController wristPID = new PIDController(.5, 0, 0);
  private SimpleMotorFeedforward m_wristFeedForward = new SimpleMotorFeedforward(0,0,0);

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    

    m_wristMotor = new WPI_TalonFX(Constants.WristConstants.kWristID);
    m_wristMotor.setNeutralMode(NeutralMode.Brake); //DO NOT CHANGE FROM BRAKE
    m_wristMotor.setSelectedSensorPosition(0);
  }

  public void setWrist(double percentOutput) {
    m_wristMotor.set(ControlMode.PercentOutput, percentOutput);
  }
  public double getPosition(){
    return m_wristMotor.getSelectedSensorPosition();
  }
  public double getDriveRate(){
    return m_wristMotor.getSelectedSensorVelocity();
    }
  public double pidError(){
    return wristPID.getPositionError();
  }
  public void maintainPosition(){
    double currentPosition = getPosition();
    double outputWrist = wristPID.calculate(currentPosition, currentPosition);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotor.setVoltage(outputWrist + wristFeedForward);
    
  }

  public void coneIntakeAngled(){
    double currentPosition = getPosition();
    double outputWrist = wristPID.calculate(currentPosition, Constants.WristConstants.kWristConeSetpoint);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotor.setVoltage(outputWrist + wristFeedForward);
    

  //Wrist Angled for Cone intake
  }
  public void coneOuttakeAngled(){
    double currentPosition = getPosition();
    double outputWrist = wristPID.calculate(currentPosition, Constants.WristConstants.kWristConeOuttakeSetpoint);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotor.setVoltage(outputWrist + wristFeedForward);
  }

  public void cubeAngled(double wristPoint){
    double currentPosition = getPosition();
    double outputWrist = wristPID.calculate(currentPosition, wristPoint);
        double wristFeedForward = m_wristFeedForward.calculate(getDriveRate());
        m_wristMotor.setVoltage(outputWrist + wristFeedForward);
  //Wrist Angled for Cube intake
  }

  @Override
  public void periodic() {
    //placed here because i need it updated constantly and dont want to deal with putting it in a command properly
    if (RobotContainer.m_controller.getLeftBumperPressed()) {
        if (bumperPos == 3){
            bumperPos= 3;
        }
        else{
            bumperPos++;
        }
    }
    if (RobotContainer.m_controller.getRightBumperPressed()) {
        if (bumperPos == 0){
            bumperPos= 0;
        }
        else{
            bumperPos =- 1;
        }
    }
    if (RobotContainer.m_controller.getYButton()) {
        bumperPos = 0;
    }
    if (RobotContainer.m_controller.getXButton()){
        bumperPos = 3;
    }

    // This method will be called once per scheduler run
  }
}

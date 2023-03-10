// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SlideSubsystem extends SubsystemBase {
    private WPI_TalonFX m_rightSlide, m_leftSlide, m_tilt;
    
    
    private SimpleMotorFeedforward m_slideFeedForward = new SimpleMotorFeedforward(0,0,0);
   


    @SuppressWarnings("unused")
    private PIDController 
        controllerRight, controllerLeft = new PIDController(.5, 0, 0),
        controllerTilt = new PIDController(.5, 0, 0);
        
    // maybe add a feed forward? May be unnecessary though
    /** Creates a new SlideSubsystem. */
    public SlideSubsystem() {
        m_leftSlide = new WPI_TalonFX(Constants.SlideConstants.kLSlideID);
        m_leftSlide.setNeutralMode(NeutralMode.Brake);
        m_leftSlide.setSelectedSensorPosition(0);

        m_rightSlide = new WPI_TalonFX(Constants.SlideConstants.kRSlideID);
        m_rightSlide.setNeutralMode(NeutralMode.Brake);
        m_rightSlide.setSelectedSensorPosition(0);

        m_tilt = new WPI_TalonFX(Constants.SlideConstants.kTiltID);
        m_tilt.setNeutralMode(NeutralMode.Brake); //DO NOT CHANGE FROM BRAKE
        m_tilt.setSelectedSensorPosition(0);

        m_leftSlide.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
    public double getDriveRate(){
        return m_leftSlide.getSelectedSensorVelocity();
    }
    public void    moveSlide(double desiredState){
        double currentPosition = getLeftPosition();
        double output = controllerLeft.calculate(currentPosition, desiredState);
        double slideFeedForward = m_slideFeedForward.calculate(getDriveRate());
        m_leftSlide.setVoltage(output + slideFeedForward);

    }

    public void setSlide(double percentOutput) {
        m_leftSlide.set(ControlMode.PercentOutput, percentOutput);
        m_rightSlide.set(ControlMode.PercentOutput, -percentOutput);
      
    }

    public void setTilt(double percentOutput) {
        m_tilt.set(ControlMode.PercentOutput, percentOutput);
    }
    public double getLeftPosition(){
        return m_leftSlide.getSelectedSensorPosition() * Constants.SlideConstants.kSlideConstant;

    }
    public double getRightPosition(){
        return m_rightSlide.getSelectedSensorPosition() * Constants.SlideConstants.kSlideConstant;

    }
    public boolean isLeftAndRightBalanced(){
        return (Math.abs(getLeftPosition()-getRightPosition()) < .01);
    }
    
    public double getSlideOutput(){
        return 0; //TODO              

    }



    // make a function that gets the number of ticks
    // make a functon that set motor to number of ticks
    // make a function that designates levels for game
    // try to optimize
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Balance", isLeftAndRightBalanced());
        // This method will be called once per scheduler run
    }
}

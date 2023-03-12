// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class IntakeOuttakeProcessWrist extends CommandBase {
  boolean isPieceIntaken;
  ClawSubsystem m_claw;
  SlideSubsystem m_slide;
  WristSubsystem m_wrist;
  boolean isCube;
  double setpoint;
  boolean isDone = false;
  /** Creates a new IntakeOuttakeProcessCube. */
  public IntakeOuttakeProcessWrist(WristSubsystem wrist, boolean isCube, ClawSubsystem claw) {
    this.isPieceIntaken = claw.isPieceIntaken();
    this.m_wrist = wrist;
    this.isCube = isCube;
    addRequirements(m_wrist);

  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isPieceIntaken && isCube){
      m_wrist.cubeAngled(Constants.WristConstants.kWristCubeSetpoint[0]);
      if (Math.abs(m_wrist.pidError()) < .1){
        isDone = true;
      }
      
      

    }else if(isPieceIntaken && !isCube){
      m_wrist.coneIntakeAngled(); //might need driver control here, in case cone is in odd position
      if (Math.abs(m_wrist.pidError()) < .1){
        isDone = true;
      }

    }else if (!isPieceIntaken && isCube){
      m_wrist.cubeAngled(Constants.WristConstants.kWristCubeSetpoint[m_wrist.bumperPos + 1]);
      if (Math.abs(m_wrist.pidError()) < .1){
        isDone = true;
      }

    }else if (!isPieceIntaken && !isCube){
      m_wrist.coneOuttakeAngled();
      if (Math.abs(m_wrist.pidError()) < .1){
        isDone = true;
      }

    }
  
  
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.m_wristMotor.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    return isDone;
    
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class WristConeIntake extends CommandBase {
    boolean isPieceIntaken;
    ClawSubsystem m_claw;
    SlideSubsystem m_slide;
    WristSubsystem m_wrist;
    boolean isCube;
    double setpoint;
    boolean isDone = false;
    RobotContainer m_RobotContainer;
    Lights lightstrip;
    /** Creates a new IntakeOuttakeProcessCube. */
    public WristConeIntake(WristSubsystem wrist, boolean isCube, ClawSubsystem claw, RobotContainer m_RobotContainer) {
        this.isPieceIntaken = claw.isPieceIntaken();
        this.m_wrist = wrist;
        this.isCube = isCube;
        this.m_RobotContainer = m_RobotContainer;
        this.lightstrip = m_RobotContainer.lightstrip;   
        lightstrip.scheduler.setLightEffect(() -> {
            lightstrip.flashingRGB(isCube ? 255 : 120, isCube ? 210 : 0, isCube ? 0 : 153);
        }, 1.5, 10, .14);
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
      

      
         
            m_wrist.coneIntakeAngled();
            if (Math.abs(m_wrist.pidError()) < .1){
                isDone = true;
                m_RobotContainer.intakeFinish = true;
            } 
           
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        m_RobotContainer.intakeFinish = false;
        m_wrist.m_wristMotorFirst.setVoltage(0);
    m_wrist.m_wristMotorSecond.setVoltage(0);
    
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() { 
        return false;  
    }
}

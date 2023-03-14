// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.CurrentSpikeCounter;

public class ClawCubeIntake extends CommandBase {
    public ClawSubsystem clawSubsystem;
    public CurrentSpikeCounter spikeCounter;
    public boolean finished = false;
    public WristSubsystem wrist;
    public boolean isDoneCheck;
    public double test;
    public boolean intakeOrOuttake;
    public boolean isCube;
    public Timer timer;
    public double time;

    public RobotContainer robotContainer;
        // these will be the heights of the slide at different points. The height will be set as ClawConstants.ClawSetpoints[bumperPos]
    
    /** Creates a new SlideIntakeAndOuttakeCommand. */
    public ClawCubeIntake( ClawSubsystem clawSubsystem, WristSubsystem wrist, boolean isCube, RobotContainer robotContainer, boolean isIntaking) {
        this.clawSubsystem = clawSubsystem;
        this.wrist = wrist; 
        this.robotContainer = robotContainer;
        this.intakeOrOuttake = isIntaking;
        
       
        
        
        
        // Use addRequirements() here to declare subsystem dependencies.
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      time = Timer.getFPGATimestamp();
      isDoneCheck = clawSubsystem.isPieceIntaken();
      test = 0;
  
        
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
      
      SmartDashboard.putBoolean("abdjasbdjas", isDoneCheck);
     
        // Bejamin since this is in execute() should this be a while loop? Might stall the code/ hog cpu temporarily
        //you right. it is also spelled Bangiman.
        //Liam i think you edited my name there. its benjamin. not bangiman or bejamin
        if (clawSubsystem.isPieceIntaken() == isDoneCheck){
            clawSubsystem.setIntake(-.45);
            
        }  else {
            robotContainer.intakeFinish = true;
        }
   
         
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        robotContainer.intakeFinish = false;
        clawSubsystem.setIntake(0);
        robotContainer.inOrOut += 1;
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

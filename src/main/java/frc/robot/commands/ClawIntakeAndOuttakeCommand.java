// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.CurrentSpikeCounter;

public class ClawIntakeAndOuttakeCommand extends CommandBase {
    public ClawSubsystem clawSubsystem;
    public CurrentSpikeCounter spikeCounter;
    public boolean finished = false;
    public WristSubsystem wrist;
    public boolean isDoneCheck;
    public static double[] Setpoints = { 
        0,
        0,
        0,
        0
        // these will be the heights of the slide at different points. The height will be set as Setpoints[bumperPos]
    }; // TODO: put thes in constants
    
    /** Creates a new SlideIntakeAndOuttakeCommand. */
    public ClawIntakeAndOuttakeCommand( ClawSubsystem clawSubsystem, WristSubsystem wrist) {
        this.clawSubsystem = clawSubsystem;
        this.wrist = wrist; 
        isDoneCheck = clawSubsystem.isPieceIntaken(); // TODO: This may not update constantly because its not in execute(), it is only called once in the constructor.
        
        addRequirements(clawSubsystem);
        
        // Use addRequirements() here to declare subsystem dependencies.
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // Bejamin since this is in execute() should this be a while loop? Might stall the code/ hog cpu temporarily
        //you right. it is also spelled Banjiman.
        if (clawSubsystem.isPieceIntaken() == isDoneCheck && clawSubsystem.isPieceIntaken() == false){
            clawSubsystem.setIntake(.1);
            
        } if (clawSubsystem.isPieceIntaken() == isDoneCheck && clawSubsystem.isPieceIntaken() == true){
            clawSubsystem.setIntake(.4);
            
        } else {
            finished = true;
        }
         
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setIntake(0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}

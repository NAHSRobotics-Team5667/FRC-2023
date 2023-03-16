// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase {
    public ClawSubsystem claw;
    boolean done;
    int counter = 0;
    double isCube;
    double meWhenLiam = -1000000;
    RobotContainer robotContainer;
    /** Creates a new ClawCommand. */
    public ClawCommand(ClawSubsystem claw, RobotContainer robotContainer) {
        this.claw = claw;
        this.robotContainer = robotContainer;
        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        claw.setIntake(0);
        isCube = robotContainer.coneOrCubeBoolean() ? -1 : 1;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        claw.setIntake(0);
        // if (claw.getMotorInput() > 0 && !(claw.getMotorSpeed() == 0)) {
        //     claw.setIntake(-1);
        // } else if (claw.getMotorInput() > 0 && (claw.getMotorSpeed() == 0)) {
        //     claw.setIntake(0);    // }
    
//     if (claw.isPieceIntaken() || counter > 0){
//         counter += 1;
        
//         if (claw.getPosition() < meWhenLiam){

        


//         claw.setIntake(.075*isCube);
//         }
//     } else {
//     counter += 1;
        
//     if (claw.getPosition() > meWhenLiam){

    


//     claw.setIntake(-.075*isCube);
//     }
// } 


//     if (counter == 1){
//        meWhenLiam = claw.getPosition(); 
//     }
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        @SuppressWarnings("unused")
        double meWhenLiam = 0;
        this.counter = 0;
        claw.setIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
//sup raf, hows spring break?

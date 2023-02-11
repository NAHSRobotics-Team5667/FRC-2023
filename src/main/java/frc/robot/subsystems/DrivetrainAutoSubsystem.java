// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainAutoSubsystem extends DrivetrainSubsystem {
	public DrivetrainAutoSubsystem() {
		super();
	}

    public void pleaseGodLetThisWork(SwerveModuleState Wheel1, SwerveModuleState Wheel2, SwerveModuleState Wheel3, SwerveModuleState Wheel4){
        m_frontLeft.setDesiredState(Wheel1);
		m_frontRight.setDesiredState(Wheel2);
		m_backLeft.setDesiredState(Wheel3);
		m_backRight.setDesiredState(Wheel4);
    }

	

}
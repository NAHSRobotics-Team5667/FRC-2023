// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndOuttakeProcedure extends SequentialCommandGroup {
  RobotContainer m_RobotContainer;
  boolean isCube;
  BooleanSupplier getSticks;
  /** Creates a new IntakeAndOuttakeProcedure. */
  public IntakeAndOuttakeProcedure(RobotContainer m_RobotContainer, boolean isCube) {
    BooleanSupplier getSticks = new BooleanSupplier() {
      
      public boolean getAsBoolean() {
        return ((MathUtil.applyDeadband(-RobotContainer.m_controller.getLeftX(), 0.1))> 0) || ((MathUtil.applyDeadband(-RobotContainer.m_controller.getLeftY(), 0.1))> 0);
      }
  };
    this.isCube = isCube;
    this.m_RobotContainer = m_RobotContainer;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new IntakeOuttakeProcessWrist(m_RobotContainer.m_wrist, isCube, m_RobotContainer.m_claw), new ClawIntakeAndOuttakeCommand(m_RobotContainer.m_claw, m_RobotContainer.m_wrist));
    until(getSticks);
  }
}

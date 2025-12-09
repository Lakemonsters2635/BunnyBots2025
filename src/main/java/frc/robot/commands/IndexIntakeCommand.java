// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexIntakeCommand extends Command {
  private IndexSubsystem m_indexSubsystem;
  
  /** Creates a new IndexIntakeCommand. */
  public IndexIntakeCommand(IndexSubsystem indexSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexSubsystem = indexSubsystem;
    addRequirements(m_indexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexSubsystem.indexIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexSubsystem.stopIndex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //-49.8 = 360 degrees
    if(MathUtil.isNear(m_indexSubsystem.getTargetPos(), m_indexSubsystem.getPosition(), 10)){
      m_indexSubsystem.addToTargetPos(-90);
      return true;
    }
    return false;
  }
}

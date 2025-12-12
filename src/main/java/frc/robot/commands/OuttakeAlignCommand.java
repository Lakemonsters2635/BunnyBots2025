// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeAlignCommand extends Command {
  /** Creates a new OuttakeAlignCommand. */
  OuttakeSubsystem m_outtakeSubsystem;
  double initialPose;
  boolean isComingDown = false;
  boolean isAtBottom = false;
  Timer m_timer;
  public OuttakeAlignCommand(OuttakeSubsystem outtakeSubsystem) {
    m_outtakeSubsystem = outtakeSubsystem;
    m_timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_outtakeSubsystem.setPID(0, 0, 0);
    initialPose = m_outtakeSubsystem.getPos();
    m_outtakeSubsystem.setMotorPower(0.7);
    isComingDown = false;
    isAtBottom = false;
    m_timer.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_outtakeSubsystem.getPos() - initialPose > 1){
      m_outtakeSubsystem.setMotorPower(-2);
      m_timer.start();
      if(m_timer.get()< 0.1){
        m_outtakeSubsystem.setMotorPower(0.1);
        m_timer.stop();
        isComingDown = true;
      }
    }
    if(m_outtakeSubsystem.getPos() - initialPose < 0.5 && isComingDown){
      m_outtakeSubsystem.setMotorPower(0.22);
      isAtBottom = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_outtakeSubsystem.getPos()-initialPose < 0.1 && isAtBottom){
      return true;
    }
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.OuttakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeRemoveBacklash extends Command {
  /** Creates a new OuttakeRemoveBacklash. */
  OuttakeSubsystem m_os;
  Timer timer;
  public OuttakeRemoveBacklash(OuttakeSubsystem os) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_os = os;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_os.setPID(0,0,0);
    m_os.setVoltage(.2);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_os.setVoltage(0);
    timer.stop();
    m_os.setPID(Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > .5){
      return true;
    }
    return false;
  }
}

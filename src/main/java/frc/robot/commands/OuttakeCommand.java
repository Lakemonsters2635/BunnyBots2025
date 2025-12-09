// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.OuttakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeCommand extends Command {
  /** Creates a new OuttakeCommand. */
  
  public OuttakeSubsystem m_outtakeSubsystem;
  private Timer timer;
  
  public OuttakeCommand(OuttakeSubsystem outtakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_outtakeSubsystem = outtakeSubsystem;
    timer = new Timer();
    addRequirements(m_outtakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_outtakeSubsystem.setPID(Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D);
    m_outtakeSubsystem.setAngle(Constants.SHOOTER_TARGET_DELTA_ANGLE);
    timer.reset();
    timer.start();
    // SmartDashboard.putBoolean("isFin", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_outtakeSubsystem.setPID(0, 0, 0);
    timer.stop();
  }

  // TODO: Move to a math tools library, see the commented in isFinished()
  public boolean isCloseTo(double target, double ref, double threshold){
    return Math.abs(target - ref) < threshold;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: change limit value(0) later when tested
    // if(MathUtil.isNear(Constants.SHOOTER_TARGET_DELTA_ANGLE, m_outtakeSubsystem.getPosCalibrated(),  .2)
    //     && (timer.get() > .2)){
    //   SmartDashboard.putBoolean("isFin", true);
    //   return true;
    // }

    if( isCloseTo(Constants.SHOOTER_TARGET_DELTA_ANGLE, m_outtakeSubsystem.getPosCalibrated(),  .2)
        && (timer.get() > .2)){
      // SmartDashboard.putBoolean("isFin", true);
      return true;
    }
    return false;
    
  }
}

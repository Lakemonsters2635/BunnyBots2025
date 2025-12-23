// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.OuttakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterCommand extends SequentialCommandGroup {
  /** Creates a new ShooterCommand. */
  OuttakeSubsystem m_outtakeSubsystem;

  public ShooterCommand(OuttakeSubsystem outtakeSubsystem) {
    m_outtakeSubsystem = outtakeSubsystem;

    addCommands(
        new OuttakeAlignCommand(m_outtakeSubsystem),
        new WaitCommand(0.2),
        new OuttakeCommand(outtakeSubsystem),
        new WaitCommand(0.1),
        new OuttakeBack(outtakeSubsystem));
  }

  public ShooterCommand(OuttakeSubsystem outtakeSubsystem, boolean isSingle) {
    m_outtakeSubsystem = outtakeSubsystem;

    addCommands(
        new OuttakeAlignCommand(m_outtakeSubsystem),
        new WaitCommand(0.2),
        new OuttakeCommand(outtakeSubsystem, isSingle),
        new WaitCommand(0.1),
        new OuttakeBack(outtakeSubsystem));
  }
}

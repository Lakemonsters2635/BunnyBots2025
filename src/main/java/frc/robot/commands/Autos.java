// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


public final class Autos {
  /** Example static factory for an autonomous command. */
  DrivetrainSubsystem m_dts;
  OuttakeSubsystem m_outtakeSubsystem;
  IndexSubsystem m_indexSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ObjectTrackerSubsystem m_objectTrackerSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  public Autos(DrivetrainSubsystem dts, OuttakeSubsystem outtakeSubsystem, IndexSubsystem indexSubsystem, ElevatorSubsystem elevatorSubsystem, ObjectTrackerSubsystem objectTrackerSubsystem, IntakeSubsystem intakeSubsystem) {
    m_dts = dts;
    m_outtakeSubsystem = outtakeSubsystem;
    m_indexSubsystem = indexSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_objectTrackerSubsystem = objectTrackerSubsystem;
    m_intakeSubsystem = intakeSubsystem;
  }

  public Command goStraight() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_dts.stopMotors()).withTimeout(0.1),
                new InstantCommand(() -> m_dts.setFollowJoystick(false)).withTimeout(0.1),
                m_dts.createPath(
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))),
                        new Translation2d(0, 0.5),
                        new Pose2d(0, 1, new Rotation2d(Units.degreesToRadians(-90)))),
                new InstantCommand(() -> m_dts.setFollowJoystick(true)).withTimeout(0.1),

                new InstantCommand(() -> m_dts.stopMotors()),
                new InstantCommand(() -> m_dts.resetAngle(180)),
                new InstantCommand(() -> m_dts.zeroOdometry()));
  }
  
  public Command straightScoreAuto(){
    return new SequentialCommandGroup(
      new VisionAutoCommand(m_dts,m_objectTrackerSubsystem , 10, 6, -58.5, 0, -90),
      new WaitCommand(2),
      new ShooterCommand(m_outtakeSubsystem),
      new WaitCommand(2),
      new IndexIntakeCommand(m_indexSubsystem),
      new WaitCommand(2),
      new ShooterCommand(m_outtakeSubsystem)
    );
  }
  
  public Command leftScoreAuto(){
    return new SequentialCommandGroup(
      new PidAutoCommand(m_dts, m_objectTrackerSubsystem, 0, -Units.inchesToMeters(148.375-11) , 0),
      new WaitCommand(1)
      ,new VisionAutoCommand(m_dts, m_objectTrackerSubsystem, 2, 6, -58.5 + 7, 0, -90),
      new WaitCommand(2),
      new ShooterCommand(m_outtakeSubsystem),
      new WaitCommand(2),
      new IndexIntakeCommand(m_indexSubsystem),
      new WaitCommand(2),
      new ShooterCommand(m_outtakeSubsystem)    
    );
  }

  public Command rightScoreAuto(){
    return new SequentialCommandGroup(
      new PidAutoCommand(m_dts, m_objectTrackerSubsystem, 0, Units.inchesToMeters(148.375-11) , 0),
      new WaitCommand(1),
      new VisionAutoCommand(m_dts, m_objectTrackerSubsystem, 5, 6, -58.5+2, 0, -90),
      new WaitCommand(2),
      new ShooterCommand(m_outtakeSubsystem),
      new WaitCommand(2),
      new IndexIntakeCommand(m_indexSubsystem),
      new WaitCommand(2),
      new ShooterCommand(m_outtakeSubsystem)       
    );
  }
}

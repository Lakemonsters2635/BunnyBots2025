// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import frc.robot.subsystems.VisionLocalizationSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // CONTROLLERS (eg. joystick, streamdeck, etc...)
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);

  // SUBSYSTEMS
  public static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  public static ObjectTrackerSubsystem m_objectTrackerSubsystem =
      new ObjectTrackerSubsystem("Eclipse");
  public static VisionLocalizationSubsystem m_visionLocalizationSubsystem =
      new VisionLocalizationSubsystem(m_objectTrackerSubsystem, m_drivetrainSubsystem);

  // COMMANDS
  // public static VisionAutoCommand m_visionAutoCommand = new
  // VisionAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 8, 5, -24, 0.0001,90);

  public static Autos m_autos = new Autos(m_drivetrainSubsystem, m_objectTrackerSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // LEFT JOYSTICK BUTTONS

    // outtakeTrigger.onTrue(new SequentialCommandGroup(m_outtakeCommand, m_outtakeBack));
    // outtakeTrigger.onTrue(new
    // InstantCommand(()->m_outtakeSubsystem.setAngle(Constants.SHOOTER_TARGET_DELTA_ANGLE)));

    // RIGHT JOYSTICK BUTTONS
    Trigger resetSwerveButton = new JoystickButton(rightJoystick, 9);

    resetSwerveButton.onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> m_drivetrainSubsystem.resetAngle()),
            new InstantCommand(() -> m_drivetrainSubsystem.zeroOdometry())));

    // LEFT JOYSTICK BUTTON COMMANDS

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SendableChooser<Command> getAutonomousCommand() {
    SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    // SendableChooser<Command> m_alianceChooser = new SendableChooser<>();
    // An example command will be run in autonomous
    // return new VisionAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 4, 5, -24,
    // 0.0001, 270);

    // return m_autos.straightScoreAuto();
    // return m_autos.leftScoreAuto();
    // return m_autos.rightScoreAuto();
    m_autoChooser.setDefaultOption("RightAuto", m_autos.rightScoreAuto());
    m_autoChooser.addOption("MidAuto", m_autos.straightScoreAuto());
    m_autoChooser.addOption("LeftAuto", m_autos.leftScoreAuto());
    SmartDashboard.putData(m_autoChooser);
    return m_autoChooser;
    // return null;
  }
}

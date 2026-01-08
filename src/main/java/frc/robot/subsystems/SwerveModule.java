// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final SparkMaxConfig m_driveMotorConfig;
  private final SparkMaxConfig m_turningMotorConfig;

  private final AnalogInput m_turningEncoderInput;

  private double turningMotorOffsetRadians;

  private final PIDController m_drivePIDController =
      new PIDController(Constants.kPModuleDriveController, 0, 0.001);
  private final PIDController m_turningPIDController =
      new PIDController(Constants.kPModuleTurningController, 0, 0.0001);

  private double m_driveMotorGain;
  private int driveId = 0;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param analogEncoderPort Analog input port for the turning encoder.
   * @param turningMotorOffsetRadians Offset to add to the turning encoder reading to align with the
   *     module's zero position.
   * @param driveMotorGain Gain to apply to the drive motor output for tuning.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int analogEncoderPort,
      double turningMotorOffsetRadians,
      double driveMotorGain) {

    driveId = driveMotorChannel;
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotorConfig = new SparkMaxConfig();
    m_turningMotorConfig = new SparkMaxConfig();

    m_driveMotorConfig.idleMode(IdleMode.kBrake);
    m_turningMotorConfig.idleMode(IdleMode.kBrake);

    m_driveMotor.configure(
        m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_turningMotor.configure(
        m_turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.turningMotorOffsetRadians = turningMotorOffsetRadians;

    m_driveMotorGain = driveMotorGain;

    m_turningEncoderInput = new AnalogInput(analogEncoderPort);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getEncoder().getPosition() * Constants.kDriveEncoderDistancePerPulse,
        new Rotation2d(getTurningEncoderRadians()));
  }

  public double getDrivePosition() {
    return m_driveMotor.getEncoder().getPosition();
  }

  public double getTurningEncoderVoltage() {
    return m_turningEncoderInput.getVoltage();
  }

  /*
   * Ranges from -PI to PI
   * I radians
   */
  public double getTurningEncoderRadians() {
    double angle =
        (1.0 - (getTurningEncoderVoltage() / RobotController.getVoltage5V())) * 2.0 * Math.PI
            + turningMotorOffsetRadians;
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }

    return angle - Math.PI;
  }

  public double getVelocity() {
    return m_driveMotor.getEncoder().getVelocity() * (Constants.kDriveEncoderDistancePerPulse / 60);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), new Rotation2d(getTurningEncoderRadians()));
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = desiredState;

    // Prevent rotating module if speed is small. Prevents Jittering.
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state.optimize(new Rotation2d(getTurningEncoderRadians()));

    // Calculate the drive output from the drive PID controller.
    // Note: due to the drive PID constants being zero currently, this driveOutput will
    //       always be zero.
    final double driveOutput =
        m_drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);

    final double driveFeedForward = state.speedMetersPerSecond / Constants.kMaxSpeedMetersPerSecond;

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(getTurningEncoderRadians(), state.angle.getRadians());

    SmartDashboard.putNumber(
        "angleSwerve" + Integer.toString(driveId), state.angle.getRadians()); //
    SmartDashboard.putNumber(
        "cAngleSwerve" + Integer.toString(driveId), getTurningEncoderRadians()); //

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(
        MathUtil.clamp(
            (driveOutput + driveFeedForward) * m_driveMotorGain, // gain = 1, no gain
            -1.0, // min -100%
            1.0 // max +100%
            ));
    m_turningMotor.set(turnOutput);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {
  /** Creates a new IndexSubsystem. */
  private final SparkMax m_indexMotor;

  private final SparkMaxConfig m_indexMotorConfig;

  private double targetPos = -90;

  public IndexSubsystem() {
    m_indexMotor = new SparkMax(Constants.INDEX_SUBSYSTEM_ID, MotorType.kBrushless);
    m_indexMotorConfig = new SparkMaxConfig();
    m_indexMotorConfig.idleMode(IdleMode.kBrake);

    m_indexMotor.getEncoder().setPosition(0);

    m_indexMotor.configure(
        m_indexMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public void indexIntake() {
    m_indexMotor.setVoltage(Constants.INDEX_SUBSYSTEM_VOLTAGE);
  }

  public void stopIndex() {
    m_indexMotor.setVoltage(0);
  }

  public double getPosition() {
    // -49.8 = 360 degrees
    return m_indexMotor.getEncoder().getPosition() / 50 * 360;
  }

  public void setEncoderPos(double encoderPos) {
    m_indexMotor.getEncoder().setPosition(encoderPos);
  }

  public void addToTargetPos(double add) {
    targetPos += add;
  }

  public double getTargetPos() {
    return targetPos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Index pos", getPosition());
  }
}

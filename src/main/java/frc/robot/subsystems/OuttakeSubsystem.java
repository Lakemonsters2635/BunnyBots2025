// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OuttakeSubsystem extends SubsystemBase {
  /** Creates a new OuttakeSubsystem. */

  // PhoenixPIDController pidController = new PhoenixPIDController(0,0,0);

  Slot0Configs slot0Configs = new Slot0Configs();

  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  public double initialEncoderValue;
  double targetPose;
  TalonFX outTakeMotor;
  VoltageConfigs voltageConfigs;

  Joystick m_joystick = new Joystick(0);

  public OuttakeSubsystem() {
    outTakeMotor = new TalonFX(13);
    voltageConfigs = new VoltageConfigs();
    voltageConfigs.SupplyVoltageTimeConstant = 9;
    initialEncoderValue = outTakeMotor.getPosition().getValueAsDouble();
    outTakeMotor.getConfigurator().apply(voltageConfigs);
    slot0Configs.kG = 0;
    // setPID(0, 0, 0);
    setPID(Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D);
    SmartDashboard.putNumber("initialOuttakePos", initialEncoderValue);
  }

  public void setPID(double p, double i, double d) {
    slot0Configs.kP = p;
    slot0Configs.kI = i;
    slot0Configs.kD = d;
    outTakeMotor.getConfigurator().apply(slot0Configs);
  }

  public void setAngle(double deltaPos) {
    // in raw encoder counts
    targetPose = deltaPos + initialEncoderValue;
    outTakeMotor.setControl(m_request.withPosition(targetPose));
  }

  public double getTargetPos() {
    return Constants.SHOOTER_TARGET_DELTA_ANGLE;
  }

  public void motorOutTake() {
    outTakeMotor.setVoltage(5);
  }

  public void setMotorPower(double motorPower) {
    outTakeMotor.setVoltage(motorPower);
  }

  public void motorMoveBack() {
    outTakeMotor.setVoltage(-1);
  }

  public void setVoltage(double volts) {
    outTakeMotor.setVoltage(volts);
  }

  public void stopmotorOutTake() {
    outTakeMotor.setVoltage(0.20); // To avoid arm crushing into
  }

  public double getPos() {
    return outTakeMotor.getPosition().getValueAsDouble();
  }

  public double getPosCalibrated() {
    return outTakeMotor.getPosition().getValueAsDouble() - initialEncoderValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Outtake Position", getPos());
    SmartDashboard.putNumber("Outtake Position Cal", getPosCalibrated());
    // SmartDashboard.putNumber("outtakeMotorVel", outTakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "outtakeMotorVoltage", outTakeMotor.getMotorVoltage().getValueAsDouble());
    // SmartDashboard.putNumber("OuttakeCurrentSupply",
    // outTakeMotor.getSupplyCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("OuttakeCurrentStator",
    // outTakeMotor.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("outtakeRPM", outTakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("target", Constants.SHOOTER_TARGET_DELTA_ANGLE);

    // outTakeMotor.setVoltage((m_joystick.getThrottle()+1)/2);
    // SmartDashboard.putNumber("thr", (m_joystick.getThrottle()+1)/2);

  }
}

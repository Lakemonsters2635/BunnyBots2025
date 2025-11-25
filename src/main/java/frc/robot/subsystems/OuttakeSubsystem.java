// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  /** Creates a new OuttakeSubsystem. */

  // PhoenixPIDController pidController = new PhoenixPIDController(0,0,0);

  Slot0Configs slot0Configs = new Slot0Configs();
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  double initialEncoderValue;

  TalonFX outTakeMotor;

  public OuttakeSubsystem() {
    outTakeMotor = new TalonFX(12);
    initialEncoderValue = outTakeMotor.getPosition().getValueAsDouble();
    slot0Configs.kP = 15;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;
    slot0Configs.kG = 0;

    outTakeMotor.getConfigurator().apply(slot0Configs);
    outTakeMotor.setControl(m_request.withPosition(1.8 - initialEncoderValue));
  }

  public void motorOutTake() {
    outTakeMotor.setVoltage(5);
  }

  public void motorMoveBack(){
    outTakeMotor.setVoltage(-2);
  }

  public void stopmotorOutTake() {
    outTakeMotor.setVoltage(0);

  }
  public double getPos(){
    return outTakeMotor.getPosition().getValueAsDouble() - initialEncoderValue;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber( "Outtake Position", getPos());
    SmartDashboard.putNumber("outtakeMotorVel", outTakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("outtakeMotorVoltage", outTakeMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("OuttakeCurrentSupply", outTakeMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("OuttakeCurrentStator", outTakeMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("outtakeRPM", outTakeMotor.getVelocity().getValueAsDouble());
  }
}

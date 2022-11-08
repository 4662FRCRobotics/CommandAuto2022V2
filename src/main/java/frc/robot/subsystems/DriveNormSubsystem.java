// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveNormSubsystemConst;

public class DriveNormSubsystem extends SubsystemBase {
  /** Creates a new DriveNormSubsystem. */

  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_rightMotor1;
  private SparkMaxPIDController m_leftPID;
  private SparkMaxPIDController m_rightPID;

  private DifferentialDrive m_differentialDrive;

  //private SparkMaxRelativeEncoder m_leftEncoder1;
  //private SparkMaxRelativeEncoder m_rightEncoder1;

  private SimpleMotorFeedforward m_feedForward;

  private double m_driveDistance;

  public DriveNormSubsystem() {
    m_leftMotor1 = new CANSparkMax(DriveNormSubsystemConst.kLEFT_MOTOR1, MotorType.kBrushless);
    m_rightMotor1 = new CANSparkMax(DriveNormSubsystemConst.kRIGHT_MOTOR1, MotorType.kBrushless);
    resetMotor(m_leftMotor1);
    resetMotor(m_rightMotor1);
    m_leftMotor1.setInverted(DriveNormSubsystemConst.kINVERTLEFT);
    m_rightMotor1.setInverted(DriveNormSubsystemConst.kINVERTRIGHT);

    m_leftPID = m_leftMotor1.getPIDController();
    m_rightPID = m_rightMotor1.getPIDController();

    setMotorPID(m_leftPID);
    setMotorPID(m_rightPID);

    m_leftMotor1.getEncoder().setPositionConversionFactor(DriveNormSubsystemConst.kENCODER_DISTANCE_PER_PULSE_M);
    m_rightMotor1.getEncoder().setPositionConversionFactor(DriveNormSubsystemConst.kENCODER_DISTANCE_PER_PULSE_M);

    m_differentialDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

    m_feedForward = new SimpleMotorFeedforward(
      DriveNormSubsystemConst.kS_VOLTS,
      DriveNormSubsystemConst.kV_VOLT_SECOND_PER_METER,
      DriveNormSubsystemConst.kA_VOLT_SEONDS_SQUARED_PER_METER);

    System.out.printf("Encoder dist per pulse %.4f", DriveNormSubsystemConst.kENCODER_DISTANCE_PER_PULSE_M);
  }

  private void resetMotor(CANSparkMax mtrCntl) {
    mtrCntl.restoreFactoryDefaults();
    mtrCntl.setOpenLoopRampRate(DriveNormSubsystemConst.kRAMP_RATE);
    mtrCntl.setSmartCurrentLimit(DriveNormSubsystemConst.kCURRENT_LIMT);
    mtrCntl.setIdleMode(IdleMode.kBrake);
  }

  private void setMotorPID(SparkMaxPIDController mtrPID) {
    mtrPID.setP(DriveNormSubsystemConst.kCONTROL_P);
    mtrPID.setI(DriveNormSubsystemConst.kCONTROL_I);
    mtrPID.setD(DriveNormSubsystemConst.kCONTROL_D);
    mtrPID.setIZone(DriveNormSubsystemConst.kCONTROL_IZONE);
    //mtrPID.setFF(DriveNormSubsystemConst.kCONTROL_FF);
    mtrPID.setOutputRange(DriveNormSubsystemConst.kCONTROL_MIN_OUT, DriveNormSubsystemConst.kCONTROL_MAX_OUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LeftPosition", m_leftMotor1.getEncoder().getPosition());
    SmartDashboard.putNumber("RightPosition", m_rightMotor1.getEncoder().getPosition());
    SmartDashboard.putNumber("TargetPosition", m_driveDistance);
  }

  public void arcadeDrive(double forward, double rotation) {
    m_differentialDrive.feed();
    m_differentialDrive.arcadeDrive(forward, rotation);
  }

  // for trapezoidal drive - not yet working
  public void setDriveStates(TrapezoidProfile.State leftState, TrapezoidProfile.State rightState) {
    m_leftPID.setReference(
      leftState.position,
      ControlType.kPosition, 
      0,
      m_feedForward.calculate(leftState.velocity));
    m_rightPID.setReference(
      rightState.position, 
      ControlType.kPosition,
      0, 
      m_feedForward.calculate(rightState.velocity));
  }

  // for basic pid controller drive distance
  public void initDriveController(double distance) {
    
    double encoderDistance = distance / DriveNormSubsystemConst.kENCODER_DISTANCE_PER_PULSE_M;
    m_leftPID.setReference(distance, ControlType.kPosition);
    m_rightPID.setReference(distance, ControlType.kPosition);
    resetEncoders();
    m_driveDistance = encoderDistance;
  }

  public void endDriveController() {
    m_leftMotor1.stopMotor();
    m_rightMotor1.stopMotor();
  }

  public boolean isDriveAtSetpoint() {
    boolean leftOnTarget = Math.abs(m_driveDistance - m_leftMotor1.getEncoder().getPosition()) <= 0.1;
    boolean rightOnTarget = Math.abs(m_driveDistance - m_rightMotor1.getEncoder().getPosition()) <= 0.1;
    boolean comboStopped = ((Math.abs(m_leftMotor1.getEncoder().getVelocity()) + (Math.abs(m_rightMotor1.getEncoder().getVelocity()))) < 0.05);
    return (leftOnTarget && rightOnTarget || comboStopped);
  }

  public void resetEncoders() {
    m_leftMotor1.getEncoder().setPosition(0);
    m_rightMotor1.getEncoder().setPosition(0);
  }
}

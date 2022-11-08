// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveNormSubsystem;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private final DriveNormSubsystem m_driveNorm;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotate;

  public ArcadeDrive(DoubleSupplier forward, DoubleSupplier rotate, DriveNormSubsystem driveNorm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveNorm = driveNorm;
    m_forward = forward;
    m_rotate = rotate;

    addRequirements(m_driveNorm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveNorm.arcadeDrive(m_forward.getAsDouble(), m_rotate.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveNorm.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

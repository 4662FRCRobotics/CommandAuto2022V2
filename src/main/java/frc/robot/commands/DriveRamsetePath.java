// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveNormSubsystemConst;
import frc.robot.subsystems.DriveNormSubsystem;

public class DriveRamsetePath extends RamseteCommand {
  /** Creates a new DriveRamsetePath. */
  private DriveNormSubsystem m_driveNorm;
  private Trajectory m_trajectory;

  public DriveRamsetePath(Trajectory trajectory, DriveNormSubsystem driveNorm) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(trajectory,
          driveNorm::getPos,
          new RamseteController(),
          driveNorm.getSmpFeedFwd(),
          DriveNormSubsystemConst.kDRIVE_KINEMATICS,
          driveNorm::getWheelSpeeds,
          new PIDController(DriveNormSubsystemConst.kCONTROL_P, 0, DriveNormSubsystemConst.kCONTROL_D),
          new PIDController(DriveNormSubsystemConst.kCONTROL_P, 0, DriveNormSubsystemConst.kCONTROL_D),
          driveNorm::tankDriveVolts,
          driveNorm
          );
    m_trajectory = trajectory;
    m_driveNorm = driveNorm;
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_driveNorm.resetOdometry(m_trajectory.getInitialPose());
  }
  /*

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  */
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveDistanceSMPID;
import frc.robot.commands.DriveDistanceTrapProfile;
import frc.robot.commands.DriveRamsetePath;
import frc.robot.commands.WaitForCount;
import frc.robot.libraries.AutonomousCommands;
import frc.robot.libraries.AutonomousSteps;
import frc.robot.libraries.AutonomousCommandSelector;
import frc.robot.libraries.ConsoleAuto;
import frc.robot.libraries.StepState;

public class Autonomous extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  DriveNormSubsystem m_driveNorm;

  AutonomousCommandSelector<AutonomousSteps> m_autoCommand;
  String kAUTO_TAB = "Autonomous";
  String kSTATUS_PEND = "PEND";
  String kSTATUS_ACTIVE = "ACTV";
  String kSTATUS_DONE = "DONE";
  String kSTATUS_SKIP = "SKIP";
  String kSTATUS_NULL = "NULL";

  ConsoleAuto m_ConsoleAuto;
  AutonomousCommands m_autoSelectCommand[] = AutonomousCommands.values();
  AutonomousCommands m_selectedCommand;
  private ShuffleboardTab m_tab = Shuffleboard.getTab(kAUTO_TAB);
  String m_strCommand = " ";
  String m_strStepList [] = {"", "", "", "", ""};
  boolean m_bStepSWList [] = {false , false, false, false, false};
  String m_strStepStatusList [] = {"", "", "", "", ""};
  private NetworkTableEntry m_autoCmd = m_tab.add("Selected Pattern",  " ")
                                        .withSize(2, 1)
                                        .withPosition(0, 0)    
                                        .getEntry();

  private NetworkTableEntry m_iWaitLoop = m_tab.add("WaitLoop", 0)
                                        .withWidget(BuiltInWidgets.kDial)
                                        .withPosition(0, 1)
                                        .withSize(2, 2)
                                        .withProperties(Map.of("min", 0, "max", 5))
                                        .getEntry();

  private NetworkTableEntry m_step0 = m_tab.add("Step0", m_strStepList[0])
                                      .withWidget(BuiltInWidgets.kTextView)
                                      .withPosition(2, 0)
                                      .withSize(1, 1)
                                      .getEntry();
  private NetworkTableEntry m_step1 = m_tab.add("Step1", m_strStepList[1])
                                      .withWidget(BuiltInWidgets.kTextView)
                                      .withPosition(3, 0)
                                      .withSize(1, 1)
                                      .getEntry();
  private NetworkTableEntry m_step2 = m_tab.add("Step2", m_strStepList[2])
                                      .withWidget(BuiltInWidgets.kTextView)
                                      .withPosition(4, 0)
                                      .withSize(1, 1)
                                      .getEntry();
  private NetworkTableEntry m_step3 = m_tab.add("Step3", m_strStepList[3])
                                      .withWidget(BuiltInWidgets.kTextView)
                                      .withPosition(5, 0)
                                      .withSize(1, 1)
                                      .getEntry();
  private NetworkTableEntry m_step4 = m_tab.add("Step4", m_strStepList[4])
                                      .withWidget(BuiltInWidgets.kTextView)
                                      .withPosition(6, 0)
                                      .withSize(1, 1)
                                      .getEntry();
                          
  private NetworkTableEntry m_sw0 = m_tab.add("Step0Sw", m_bStepSWList[0])
                                      .withPosition(2, 1)
                                      .withSize(1, 1)
                                      .withWidget(BuiltInWidgets.kBooleanBox)
                                      .getEntry();

  private NetworkTableEntry m_sw1 = m_tab.add("Step1Sw", m_bStepSWList[1])
                                      .withPosition(3, 1)
                                      .withSize(1, 1)
                                      .withWidget(BuiltInWidgets.kBooleanBox)
                                      .getEntry();

  private NetworkTableEntry m_sw2 = m_tab.add("Step2Sw", m_bStepSWList[2])
                                      .withPosition(4, 1)
                                      .withSize(1, 1)
                                      .withWidget(BuiltInWidgets.kBooleanBox)
                                      .getEntry();

  private NetworkTableEntry m_sw3 = m_tab.add("Step3Sw", m_bStepSWList[3])
                                      .withPosition(5, 1)
                                      .withSize(1, 1)
                                      .withWidget(BuiltInWidgets.kBooleanBox)
                                      .getEntry();

  private NetworkTableEntry m_sw4 = m_tab.add("Step4Sw", m_bStepSWList[4])
                                      .withPosition(6, 1)
                                      .withSize(1, 1)
                                      .withWidget(BuiltInWidgets.kBooleanBox)
                                      .getEntry();

  private NetworkTableEntry m_st0 = m_tab.add("Stat0", m_strStepStatusList[0])
                                      .withPosition(2, 2)
                                      .withSize(1, 1)
                                      .withWidget(BuiltInWidgets.kTextView)
                                      .getEntry();

                                      
  private NetworkTableEntry m_st1 = m_tab.add("Stat1", m_strStepStatusList[1])
  .withPosition(3, 2)
  .withSize(1, 1)
  .withWidget(BuiltInWidgets.kTextView)
  .getEntry();

  
  private NetworkTableEntry m_st2 = m_tab.add("Stat2", m_strStepStatusList[2])
                                      .withPosition(4, 2)
                                      .withSize(1, 1)
                                      .withWidget(BuiltInWidgets.kTextView)
                                      .getEntry();

                                      
  private NetworkTableEntry m_st3 = m_tab.add("Stat3", m_strStepStatusList[3])
  .withPosition(5, 2)
  .withSize(1, 1)
  .withWidget(BuiltInWidgets.kTextView)
  .getEntry();

  
  private NetworkTableEntry m_st4 = m_tab.add("Stat4", m_strStepStatusList[4])
                                      .withPosition(6, 2)
                                      .withSize(1, 1)
                                      .withWidget(BuiltInWidgets.kTextView)
                                      .getEntry();

  private int m_iPatternSelect;

  private Command m_currentCommand;
  private boolean m_bIsCommandDone = false;
  private int m_stepIndex;
  private int m_iWaitCount;

  private WaitCommand m_wait1;
  private StepState m_stepWait1Sw1;
  private WaitCommand m_wait2;
  private StepState m_stepWait2Sw1;
  private StepState m_stepWait2Sw2;
  private WaitForCount m_waitForCount;
  private StepState m_stepWaitForCount;
  private DriveDistanceTrapProfile m_driveDist1;
  //private DriveDistanceTrapProfile m_driveDist2;
  private StepState m_stepDriveDist1;
  private StepState m_stepDriveDist2;
  private DriveDistanceSMPID m_driveDistSM1;
  private Trajectory m_drive3Trajectory;
  private DriveRamsetePath m_drive3Path;
  private StepState m_stepDrive3Path;

  private String m_path1JSON = "paths/Path1.wpilib.json";
  private Trajectory m_trajPath1;

  private AutonomousSteps m_currentStepName;
  private StepState[] [] m_cmdSteps;


  /*
    parameters needed
    1.  console "joystick" for autonomous
    2.  all subsystems that may be referenced by commands

    Command definitions for autonomous
    construct the command
    add to the autonomous command selector
    construct the StepState(s) for the command with optional boolean constructor
  */

  public Autonomous(ConsoleAuto consoleAuto,
                    DriveNormSubsystem driveNorm) {

    m_driveNorm = driveNorm;
    m_ConsoleAuto = consoleAuto;
    m_selectedCommand = m_autoSelectCommand[0];
    m_strCommand = m_selectedCommand.toString();
    m_autoCommand = new AutonomousCommandSelector<AutonomousSteps>();
    m_iPatternSelect = -1;

    m_wait1 = new WaitCommand(1);
    m_autoCommand.addOption(AutonomousSteps.WAIT1, m_wait1);
    //m_stepWait1Sw1 = new StepState(AutonomousSteps.WAIT1, () -> m_ConsoleAuto.getRawButton(1));
    m_stepWait1Sw1 = new StepState(AutonomousSteps.WAIT1, m_ConsoleAuto.getSwitchSupplier(1));
    
    m_wait2 = new WaitCommand(2);
    m_autoCommand.addOption(AutonomousSteps.WAIT2, m_wait2);
    m_stepWait2Sw1 = new StepState(AutonomousSteps.WAIT2, m_ConsoleAuto.getSwitchSupplier(1));
    m_stepWait2Sw2 = new StepState(AutonomousSteps.WAIT2, m_ConsoleAuto.getSwitchSupplier(2));

    m_waitForCount = new WaitForCount(1, () -> m_ConsoleAuto.getROT_SW_1());
    m_autoCommand.addOption(AutonomousSteps.WAITLOOP, m_waitForCount);
    m_stepWaitForCount = new StepState(AutonomousSteps.WAITLOOP);

    m_driveDist1 = new DriveDistanceTrapProfile(-2, m_driveNorm);
    m_autoCommand.addOption(AutonomousSteps.DRIVE1, m_driveDist1);
    m_stepDriveDist1 = new StepState(AutonomousSteps.DRIVE1, m_ConsoleAuto.getSwitchSupplier(3));
    
    m_driveDistSM1 = new DriveDistanceSMPID(-2, m_driveNorm);
    m_autoCommand.addOption(AutonomousSteps.DRIVE2, m_driveDistSM1);
    m_stepDriveDist2 = new StepState(AutonomousSteps.DRIVE2, m_stepDriveDist1.getBooleanSupplier());

    genTrajectory();
    m_drive3Path = new DriveRamsetePath(m_drive3Trajectory, m_driveNorm);
    m_autoCommand.addOption(AutonomousSteps.DRIVE3, m_drive3Path);
    m_stepDrive3Path = new StepState(AutonomousSteps.DRIVE3, m_stepDriveDist1.getBooleanSupplier());

    readPaths();

    m_cmdSteps = new StepState [] [] {
      {m_stepWaitForCount, m_stepDriveDist1, m_stepWait1Sw1, m_stepWait2Sw2},
      {m_stepWait2Sw1, m_stepDriveDist2, m_stepWaitForCount},
      {m_stepWaitForCount, m_stepDrive3Path, m_stepWait2Sw1}
    };

  }

  private void genTrajectory() {
    m_drive3Trajectory = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 0)),
        new Pose2d(4, 0, new Rotation2d(0)),
        m_driveNorm.getTrajConfig());
  }

  private void readPaths() {
    try {
      Path trajPath1 = Filesystem.getDeployDirectory().toPath().resolve(m_path1JSON);
      m_trajPath1 = TrajectoryUtil.fromPathweaverJson(trajPath1);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + m_path1JSON, ex.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_autoCmd.setString(m_strCommand);
    m_iWaitLoop.setValue(m_iWaitCount);
    m_step0.setString(m_strStepList[0]);
    m_sw0.setValue(m_bStepSWList[0]);
    m_st0.setString(m_strStepStatusList[0]);
    m_step1.setString(m_strStepList[1]);
    m_sw1.setValue(m_bStepSWList[1]);
    m_st1.setString(m_strStepStatusList[1]);
    m_step2.setString(m_strStepList[2]);
    m_sw2.setValue(m_bStepSWList[2]);
    m_st2.setString(m_strStepStatusList[2]);
    m_step3.setString(m_strStepList[3]);
    m_sw3.setValue(m_bStepSWList[3]);
    m_st3.setString(m_strStepStatusList[3]);
    m_step4.setString(m_strStepList[4]);
    m_sw4.setValue(m_bStepSWList[4]);
    m_st4.setString(m_strStepStatusList[4]);
  }

  public void selectAutoCommand() {

    int autoSelectIx = m_ConsoleAuto.getROT_SW_0();
    m_iPatternSelect = autoSelectIx;
    if (autoSelectIx >= m_autoSelectCommand.length) {
      autoSelectIx = 0;
    }
    m_selectedCommand = m_autoSelectCommand[autoSelectIx];
    m_strCommand = m_selectedCommand.toString();
    for (int ix=0; ix < m_cmdSteps [autoSelectIx].length; ix++) {
      m_strStepList [ix] = m_cmdSteps [autoSelectIx] [ix].getStrName();
      m_bStepSWList [ix] = m_cmdSteps [autoSelectIx] [ix].isTrue();
      m_strStepStatusList [ix] = kSTATUS_PEND;
    }
    for (int ix = m_cmdSteps [autoSelectIx].length; ix < m_strStepList.length; ix++) {
      m_strStepList [ix] = "";
      m_bStepSWList [ix] = false;
      m_strStepStatusList [ix] = "";
    }
    m_iWaitCount = m_ConsoleAuto.getROT_SW_1();

  }

  public void initGetCommand() {
    m_stepIndex = -1;
    
  }

  public Command getNextCommand() {
    m_currentStepName = null;
    m_currentCommand = null;
    String completionAction = kSTATUS_DONE;

    while (m_currentCommand == null && !m_bIsCommandDone) {
      m_currentStepName = getNextActiveCommand(completionAction);
      if (m_currentStepName != null) {
        m_currentCommand = m_autoCommand.getSelected(m_currentStepName);
        if (m_currentCommand == null) {
          completionAction = kSTATUS_NULL;
        }
      }
    }
    return m_currentCommand;
  }

  // gets the next available command
  private AutonomousSteps getNextActiveCommand(String completionAction) {

    // System.out.println("getNextActiveCommand");

    AutonomousSteps stepName = null;

    while (stepName == null  && !m_bIsCommandDone) {
      if (m_stepIndex >= 0) {
        m_strStepStatusList [m_stepIndex] = completionAction;
      }
      m_stepIndex++;
      if (m_stepIndex >= m_cmdSteps [m_iPatternSelect].length) {
        m_bIsCommandDone = true;
      } else {
        if (m_cmdSteps [m_iPatternSelect] [m_stepIndex].isTrue()) {
          m_strStepStatusList [m_stepIndex] = kSTATUS_ACTIVE;
          stepName = m_cmdSteps [m_iPatternSelect] [m_stepIndex].getName();
        } else {
          completionAction = kSTATUS_SKIP;
        }
      }
    }
    
    return stepName;
  }

  public boolean isCommandDone() {
    return m_bIsCommandDone;
  }

}

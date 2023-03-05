// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class ShiftTankDrive extends SubsystemBase {
  /** Creates a new Shifting Tank Drive. */
  private final WPI_VictorSPX m_leftDrive1 = new WPI_VictorSPX(DriveConstants.kFrontLeftDrive);
  private final WPI_VictorSPX m_leftDrive2 = new WPI_VictorSPX(DriveConstants.kBackLeftDrive);
  private final WPI_VictorSPX m_rightDrive1 = new WPI_VictorSPX(DriveConstants.kFrontRightDrive);
  private final WPI_VictorSPX m_rightDrive2 = new WPI_VictorSPX(DriveConstants.kBackRightDrive);
  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftDrive1, m_leftDrive2);
  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightDrive1, m_rightDrive2);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  // Drive train base is for a tank style mechanical build.

  // Double Solenoids added for shifting the gear boxes on right and left side.
  private final DoubleSolenoid m_shifterL = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    Constants.ShiftConstants.kSolenoidPorts[0], 
    Constants.ShiftConstants.kSolenoidPorts[1]);
  private final DoubleSolenoid m_shifterR = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    Constants.ShiftConstants.kSolenoidPorts[2], 
    Constants.ShiftConstants.kSolenoidPorts[3]);


  public ShiftTankDrive() {
    m_rightDrive.setInverted(true);
  }

  /**
   * 
   * @param fwd
   * @param rot
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  } // Establish arcade drive.

  // Actions for shifting between high and low speeds.
  public void shiftHigh() {
    m_shifterL.set(kForward);
    m_shifterR.set(kForward);
  }

  public void shiftLow() {
    m_shifterL.set(kReverse);
    m_shifterR.set(kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

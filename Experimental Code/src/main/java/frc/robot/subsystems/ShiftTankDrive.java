// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class ShiftTankDrive extends SubsystemBase {
  /** Creates a new Shifting Tank Drive. */
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);
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
  }

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

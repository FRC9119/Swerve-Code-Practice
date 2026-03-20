// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FuelConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Targeting;

public class CANFuelSubsystem extends SubsystemBase {
  private final WPI_TalonSRX feederRoller;
  private final TalonFX launcherRoller;
  private final TalonFX launcherSecondary;
  private final TalonFX intakeRoller;
  private final CommandSwerveDrivetrain drivetrain;
  public PIDController launchPID;
  public SimpleMotorFeedforward launchFF = new SimpleMotorFeedforward(0.14789,0.11835,0);
  public SysIdRoutine sysIdFlywheelRoutine;
  public double setpointRPS = DEFAULT_LAUNCH_RPM/60;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // create brushed motors for each of the motors on the launcher mechanism
    launcherRoller = new TalonFX(LAUNCHER_MOTOR_ID_1);
    launcherSecondary = new TalonFX(LAUNCHER_MOTOR_ID_2);
    launcherSecondary.setControl(new Follower(LAUNCHER_MOTOR_ID_1, MotorAlignmentValue.Aligned));

    feederRoller = new WPI_TalonSRX(FEEDER_MOTOR_ID);
    intakeRoller = new TalonFX(INTAKE_MOTOR_ID);
    // create config for feeder CIM and apply it
    TalonSRXConfiguration feederConfig = new TalonSRXConfiguration();
    feederConfig.peakCurrentLimit = FEEDER_MOTOR_CURRENT_LIMIT;
    feederRoller.configAllSettings(feederConfig);
    // Create bang-bang controller and add a setpoint (goal)
    launchPID = new PIDController(0.181,0,0);
    launchPID.setSetpoint(setpointRPS);
    // Add tolerance (amount of error that is still considered correct)
    launchPID.setTolerance(LAUNCH_TOLERANCE);
    
    
    // Add sysId routine to get PID/FF values for flywheel
    sysIdFlywheelRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> launcherRoller.setVoltage(output.magnitude()),
            null,
            this));
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // Method to unclog the shooter just in case a fuel is stuck
  public void unclog() {
    feederRoller.setVoltage(-6);
    intakeRoller
        .setVoltage(8);
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeRoller
        .setVoltage(SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    setpointRPS = Targeting.getTargetRPS(drivetrain.getPose());
    launchPID.setSetpoint(setpointRPS);
    intakeRoller.setVoltage(-6);
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    launcherRoller
        .setVoltage(launchFF.calculate(setpointRPS) + launchPID.calculate(launcherRoller.getVelocity().getValueAsDouble()));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.setVoltage(0);
    launcherRoller.setVoltage(0);
    intakeRoller.setVoltage(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    setpointRPS = Targeting.getTargetRPS(drivetrain.getPose());
    intakeRoller.setVoltage(3);
    
     launchPID.setSetpoint(setpointRPS);
    launcherRoller
        .setVoltage(launchFF.calculate(setpointRPS) + launchPID.calculate(launcherRoller.getVelocity().getValueAsDouble()));
  }

  public void launchWithoutTargeting(double rpm) {
    double rps = rpm/60;
    launchPID.setSetpoint(rps);
    launcherRoller.setVoltage(launchFF.calculate(rps) + launchPID.calculate(launcherRoller.getVelocity().getValueAsDouble()));
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intakeRoller.setVoltage(-6);

  }

  // Command factories to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  public Command launchCommand() {
    return this.run(() -> launch());
  }

  public Command unclogCommand() {
    return this.run(() -> unclog());
  }

  public Command stopCommand() {
    return this.run(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Auto;
import frc.robot.utils.ShiftTimer;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import choreo.auto.AutoChooser;

public class Dashboard extends SubsystemBase {
  public AutoChooser autoChooser;
  public SendableChooser<SysIdRoutine> sysIdRoutineChooser;

  public ShiftTimer shiftTimer = new ShiftTimer();

  public Dashboard(Auto auto, CANFuelSubsystem ballSubsystem, CommandSwerveDrivetrain drivetrain) {
    // Put a bunch of constants onto the dashboard
    SmartDashboard.putNumber("Elevating climb value", MAX_CLIMB_CURRENT);
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Default Launch RPS", DEFAULT_LAUNCH_RPS);
    SmartDashboard.putNumber("slope", 9.9);
    SmartDashboard.putNumber("intercept", 30.78);

    SmartDashboard.putNumber("Seconds to wait after center shoot before intaking", 0);
    // Make autoChooser with all autos and add it to dashboard
    autoChooser = new AutoChooser();

    autoChooser.addRoutine("One Cycle (Left)", auto::oneCycleLeft);
    autoChooser.addRoutine("One Cycle (Right)", auto::oneCycleRight);
    autoChooser.addRoutine("One Cycle (Center)", auto::oneCycleCenter);
    autoChooser.addRoutine("Two Cycle (Left)", auto::twoCycleLeft);
    autoChooser.addRoutine("Two Cycle (Right)", auto::twoCycleRight);
    autoChooser.addRoutine("Two Cycle (Center to Left)", auto::twoCycleCenterToLeft);
    autoChooser.addRoutine("Two Cycle (Center to Right)", auto::twoCycleCenterToRight);
    SmartDashboard.putData("auto", autoChooser);

    SmartDashboard.putBoolean("Can we win the auto race?", true);

    sysIdRoutineChooser = new SendableChooser<SysIdRoutine>();
    sysIdRoutineChooser.addOption("flywheel", ballSubsystem.sysIdFlywheelRoutine);
    sysIdRoutineChooser.addOption("drivetrain translation", drivetrain.m_sysIdRoutineTranslation);
    sysIdRoutineChooser.addOption("drivetrain steer", drivetrain.m_sysIdRoutineSteer);
    sysIdRoutineChooser.addOption("drivetrain rotation", drivetrain.m_sysIdRoutineRotation);

    SysIdRoutine doNothing = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((v) -> {
    }, null, drivetrain));

    sysIdRoutineChooser.setDefaultOption("nothing", doNothing);

    SmartDashboard.putData("sysId", sysIdRoutineChooser);

  }

  @Override
  public void periodic() {
    shiftTimer.updateTimer();
  }

}

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Auto;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import choreo.auto.AutoChooser;

public class Dashboard {
    public AutoChooser autoChooser;
    public SendableChooser<SysIdRoutine> sysIdRoutineChooser;

    public Dashboard(Auto auto, CANFuelSubsystem ballSubsystem, CommandSwerveDrivetrain drivetrain) {
        // Put a bunch of constants onto the dashboard
        SmartDashboard.putNumber("Elevating climb value", MAX_CLIMB_CURRENT);
        SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
        SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
        SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
        SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
        SmartDashboard.putNumber("Default Launch RPM", DEFAULT_LAUNCH_RPM);

        SmartDashboard.putNumber("Seconds to wait after center shoot before intaking", 0);
        // Make autoChooser with all autos and add it to dashboard
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Intake, shoot (from left)", auto::leftIntakeShoot);
        autoChooser.addRoutine("Intake, shoot (from right)", auto::rightIntakeShoot);
        autoChooser.addRoutine("Shoot (from center)", auto::centerShoot);
        autoChooser.addRoutine("Shoot from center, then go intake through left trench", () -> auto.centerShootSweepLeft(
                SmartDashboard.getNumber("Seconds to wait after center shoot before intaking", 0)));
        autoChooser.addRoutine("Shoot from center, then go intake through right trench",
                () -> auto.centerShootSweepRight(
                        SmartDashboard.getNumber("Seconds to wait after center shoot before intaking", 0)));
        autoChooser.addRoutine("Left two cycle (takes full field after 10s)", auto::leftTwoCycleFullField);
        autoChooser.addRoutine("Right two cycle (takes full field after 10s)", auto::rightTwoCycleFullField);
        autoChooser.addRoutine("Left two cycle (uses bump)", auto::leftTwoCycleBump);
        autoChooser.addRoutine("Right two cycle (uses bump)", auto::rightTwoCycleBump);

        SmartDashboard.putData("auto", autoChooser);

        sysIdRoutineChooser = new SendableChooser<SysIdRoutine>();
        sysIdRoutineChooser.addOption("flywheel", ballSubsystem.sysIdFlywheelRoutine);
        sysIdRoutineChooser.addOption("drivetrain translation", drivetrain.m_sysIdRoutineTranslation);
        sysIdRoutineChooser.addOption("drivetrain steer", drivetrain.m_sysIdRoutineSteer);
        sysIdRoutineChooser.addOption("drivetrain rotation", drivetrain.m_sysIdRoutineRotation);
        SmartDashboard.putData("sysId", sysIdRoutineChooser);

    }

}

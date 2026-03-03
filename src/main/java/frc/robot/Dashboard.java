package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.Constants.FuelConstants.*;

public class Dashboard {
    public static void publish() {
        SmartDashboard.putNumber("Elevating climb value", MAX_CLIMB_CURRENT);
        SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
        SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
        SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
        SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
        SmartDashboard.putNumber("Default Launch RPM", DEFAULT_LAUNCH_RPM);
    }
}

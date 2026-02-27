package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ElevatorClimber.Alignment;
import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.Constants.FuelConstants.*;

public class Dashboard {
    public Dashboard(){
        SmartDashboard.putNumber("Elevating climb value", MAX_CLIMB_CURRENT);
        
    SmartDashboard.putNumber("Default Launch RPM", DEFAULT_LAUNCH_RPM);
    }
}

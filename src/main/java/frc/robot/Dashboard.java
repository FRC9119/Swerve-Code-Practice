package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ElevatorClimber.Alignment;
import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.Constants.FuelConstants.*;

public class Dashboard {
    public final SendableChooser<Alignment> alignmentChooser;
    public Dashboard(){
        SmartDashboard.putNumber("Elevating climb value", MAX_CLIMB_CURRENT);
        alignmentChooser = new SendableChooser<Alignment>();
        alignmentChooser.addOption("left", Alignment.Left);
                alignmentChooser.addOption("right", Alignment.Right);

        SmartDashboard.putData("Alignment for tele-op climb", alignmentChooser);
    }
}

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShiftTimer {
    
    double matchTime;
    String gameData;
    boolean redFirstShift;
    boolean hubActive;
    boolean WeFirstShift;
    double remainingShiftTime;
    Optional<Alliance> alliance;
    public void updateTimer(){
       

    alliance = DriverStation.getAlliance();
    matchTime = DriverStation.getMatchTime();
    gameData = DriverStation.getGameSpecificMessage();
    
    if(gameData.isEmpty() || alliance.isEmpty())
    {
      hubActive = true;
    }
    else
    {
      if(gameData.charAt(0) == 'R')
      {
        redFirstShift = false;
      } 
      else 
      {
        redFirstShift = true;
      }

      if((alliance.get() == Alliance.Red && redFirstShift) || (alliance.get() == Alliance.Blue && !redFirstShift))
      {
        WeFirstShift = true;
      }
      else
      {
        WeFirstShift = false;
      }


      if (matchTime > 130) {
        // Transition shift, hub is active.
        remainingShiftTime = matchTime - 130;
        hubActive = true;
      } 
      else if (matchTime > 105) 
      {
        // Shift 1
        remainingShiftTime = matchTime - 105;
        hubActive = WeFirstShift;
      } 
      else if (matchTime > 80) 
      {
        // Shift 2
        remainingShiftTime = matchTime - 80;
        hubActive = !WeFirstShift;
      } 
      else if (matchTime > 55) 
      {
        // Shift 3
        remainingShiftTime = matchTime - 55;
        hubActive = WeFirstShift;
      } 
      else if (matchTime > 30) 
      {
        // Shift 4
        remainingShiftTime = matchTime - 30;
        hubActive = !WeFirstShift;
      } 
      else 
      {
        // End game, hub always active.
        remainingShiftTime = matchTime;
        hubActive = true;
      }
    }
    SmartDashboard.putNumber("shift timer", remainingShiftTime);
    SmartDashboard.putBoolean("hub active", hubActive); 
    }
}

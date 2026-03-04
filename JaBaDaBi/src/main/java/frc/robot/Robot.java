// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj.Preferences;

enum RobotName {
  DE_RONNA, 
  SAVITAR, 
  AVA, 
  RALPH,
  NOBODY;

  String asString() {
    switch (this) {
      case DE_RONNA:
        return "deRonna";
      case SAVITAR:
        return "savitara";
      case AVA:
        return "ava";
      case RALPH:
        return "ralph";
      default:
        return "nobody";
    }
  }
}

public class Robot extends LoggedRobot {
  private Command autoRoutine;
  private RobotContainer theRobotContainer;
  private Timer disabledTimer;
  private static RobotName robotName = RobotName.NOBODY;

  public void robotInit() {
       if (!Preferences.containsKey("robot")) { 
      //uncomment below to give a robot a name, then make sure to comment back!
      //ONLY UNCOMMENT ONCE FOR A SPECIFIC ROBORIO
      //Preferences.setString("robot", "deRonna"); 
    //  System.out.println("Recorded name of robot as myRobot");
      System.out.println("Robot has not been named in preferences");
    }
    else {
      String robotNameString = Preferences.getString("robot", "nobody");
      switch (robotNameString) {
        case "deRonna":
          robotName = RobotName.DE_RONNA;
          break;
        case "savitara":
          robotName = RobotName.SAVITAR;
          break;
        case "ava":
          robotName = RobotName.AVA;
          break;
        case "ralph":
          robotName = RobotName.RALPH;
          break;
        default:
          robotName = RobotName.NOBODY;
      }
     
      System.out.println("This robot is called " + robotName);
    } 
    theRobotContainer = new RobotContainer();
    disabledTimer = new Timer(); //for turning off breaking when disabled

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      // new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();
    Logger.recordOutput("Robot/name", robotName);

    if (isSimulation())  {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  public static RobotName getRobot() {
    return robotName;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    theRobotContainer.periodic();
  }

  @Override
  public void disabledInit() {
    theRobotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    // robotContainer().getLights().run(Lights.Colors.GREEN, Lights.Patterns.MARCH);
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      theRobotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }

     theRobotContainer.disabledRunningLights();
     theRobotContainer.periodic();
     theRobotContainer.initializeRobotPositionBasedOnAutoRoutine();
  }

  @Override
  public void autonomousInit() {
    theRobotContainer.setMotorBrake(true);
    autoRoutine = theRobotContainer.getAutonomousCommand();
    theRobotContainer.getShooter().reachedSpeed = false;

    if (autoRoutine != null) {
      autoRoutine.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    theRobotContainer.setMotorBrake(true);
    if (autoRoutine != null) {
      autoRoutine.cancel();
    } 
    else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit()  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}

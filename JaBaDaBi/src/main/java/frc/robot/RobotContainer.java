
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.subsystem.Autoshoot;
import frc.robot.commands.swervedrive.subsystem.Collect;
import frc.robot.commands.swervedrive.subsystem.Deploy;
import frc.robot.commands.swervedrive.subsystem.Reject;
import frc.robot.commands.swervedrive.subsystem.Shoot;
import frc.robot.commands.swervedrive.subsystem.StopAll;
import frc.robot.commands.swervedrive.subsystem.Unjam;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import java.time.Duration;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.regex.Pattern;

import org.littletonrobotics.junction.Logger;

import swervelib.SwerveInputStream;

public class RobotContainer {
  private String robotPoseHasBeenSetFor = "nothing";
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController copilotXbox = new CommandXboxController(1);
  private final SwerveSubsystem drivebase;
  private final Conveyor conveyor = new Conveyor();
  private final NeonLights lights = new NeonLights();
  private final Climber climber = new Climber();
  private final Collector collector = new Collector();
  private final Loader loader = new Loader();
  private final Shooter shooter = new Shooter(this);
  private final Sensation sensation = new Sensation();
  private final SendableChooser<Command> autoChooser;
  private boolean slowMode = false;
  private boolean rollingBack = false;

  Trigger copilotLeftStickUp = new Trigger(() -> copilotXbox.getHID().getLeftY() < -0.5);
  Trigger copilotLeftStickDown = new Trigger(() -> copilotXbox.getHID().getLeftY() > 0.5);
  Trigger copilotRightStickUp = new Trigger(() -> copilotXbox.getHID().getRightY() < -0.5);
  Trigger copilotRightStickDown = new Trigger(() -> copilotXbox.getHID().getRightY() > 0.5);

  Trigger rollingBackTrigger = new Trigger(() -> rollingBack);

  Trigger leftTOFValid = new Trigger(() -> (sensation.isLeftTOFValid() && (sensation.getLeftTOF() < 200)));
  Trigger centerTOFValid = new Trigger(() -> (sensation.isCenterTOFValid() && (sensation.getCenterTOF() < 200)));
  Trigger rightTOFValid = new Trigger(() -> (sensation.isRightTOFValid() && (sensation.getRightTOF() < 200)));

  SwerveInputStream driveAngularVelocity;
  SwerveInputStream driveFieldOriented;

  private GenericEntry loaderPowerEntry;
  private double loaderPower = 0.95;
  private double rollingBackStartTime;

  public RobotContainer() {
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/" + Robot.getRobot().asString()));
    // Driving the robot during teleOp
    driveAngularVelocity = SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> inhibitedDriveY() * -1,
        () -> inhibitedDriveX() * -1)
        .withControllerRotationAxis(rotationHandler())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8) // might be changed to 1
        .allianceRelativeControl(true)
        .cubeRotationControllerAxis(true);
    
    
    System.out.println("");

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("Collect", new Collect(collector, loader, conveyor));
    //consider replacing with just collect so no need of a command at all
    //NamedCommands.registerCommand("Collect", collector.collect());
    NamedCommands.registerCommand("RaiseClimber", climber.raiseClimber(() -> 6.5));
    NamedCommands.registerCommand("LowerClimber", climber.lowerClimber(() -> 2.0));
    NamedCommands.registerCommand("PreShoot", shooter.shoot(() -> 55).until(() -> shooter.atSpeed()));
    NamedCommands.registerCommand("Autoshoot", new Autoshoot(loader, conveyor, shooter, collector, () -> 55).withTimeout(6));
    NamedCommands.registerCommand("Shoot", new Shoot(loader, conveyor, shooter, collector, () -> 56).withTimeout(3));
    NamedCommands.registerCommand("CloseShoot", new Shoot(loader, conveyor, shooter, collector, () -> 54).withTimeout(3));
    NamedCommands.registerCommand("StopShoot", shooter.idle());
    NamedCommands.registerCommand("StopAll", new StopAll(loader, conveyor, shooter, collector, () -> 0).withTimeout(1));
    NamedCommands.registerCommand("ShootUntil", new Shoot(loader, conveyor, shooter, collector, () -> 55).until(() -> Timer.getFPGATimestamp() >= 19));
    NamedCommands.registerCommand("StopCollect", collector.intakeSetMotorPower(() -> 0));
    NamedCommands.registerCommand("Unjam", new Unjam(collector, loader, conveyor, shooter).withTimeout(1));
    NamedCommands.registerCommand("Deploy", new Deploy(collector, loader));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      Logger.recordOutput("Drive/currentPose", pose);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      Logger.recordOutput("Drive/targetPose", pose);
    });

    loaderPowerEntry = Shuffleboard.getTab("robot").add("loaderPower", loaderPower).getEntry();
  }

  private void configureBindings() {
    Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveWithAimBot = drivebase.driveWithAimBot(driveAngularVelocity);

    drivebase.setDefaultCommand(driveFieldOriented);
    if (Robot.getRobot() == Robots.SAVITAR) {
      loader.setDefaultCommand(loader.idle());
      shooter.setDefaultCommand(shooter.idle());
      conveyor.setDefaultCommand(conveyor.idle());
      collector.setDefaultCommand(collector.idle());
      climber.setDefaultCommand(climber.idle());
    }
    lights.setDefaultCommand(lights.run());

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.back().onTrue(Commands.none());
    driverXbox.b().onTrue(new InstantCommand(() -> slowMode = !slowMode)); // toggle slow mode
    driverXbox.leftBumper().whileTrue(collector.collectorAway());
    driverXbox.leftTrigger(0.1).whileTrue(collector.collectorAdjust(() -> driverXbox.getHID().getLeftTriggerAxis()));
    driverXbox.rightBumper().whileTrue(new Reject(collector, loader, conveyor, shooter));
    driverXbox.rightTrigger().whileTrue(new Collect(collector, loader, conveyor));

    // KEY BINDINGS (COPILOT)
    copilotXbox.leftBumper().whileTrue(shooter.shoot(() -> -100)
        .alongWith(loader.runLoader(() -> -loaderPower)));
    copilotXbox.leftTrigger().whileTrue(shooter.shoot(() -> 55));
    copilotXbox.rightBumper().whileTrue(new Shoot(loader, conveyor, shooter, collector, () -> 55));
    copilotRightStickUp.whileTrue(shooter.adjustHood(() -> 1.0));
    copilotRightStickDown.whileTrue(shooter.adjustHood(() -> 0.45));
    copilotLeftStickDown.whileTrue(shooter.subtractRPS());
    copilotLeftStickUp.whileTrue(shooter.addRPS());
    copilotXbox.x().whileTrue(new Shoot(loader, conveyor, shooter, collector, () -> 75));
    copilotXbox.rightTrigger().whileTrue(new Shoot(loader, conveyor, shooter, collector, () -> shooter.getShooterRPSFromDistance()));
    copilotXbox.rightStick().whileTrue(new Unjam(collector, loader, conveyor, shooter));

    copilotXbox.y().whileTrue(loader.runLoader(() -> 0.85));
    copilotXbox.b().whileTrue(driveWithAimBot);

    rollingBackTrigger.whileTrue(
      shooter.shoot(() -> 55)
      .alongWith(loader.runLoader(() -> -loaderPower))
    );

    copilotXbox.back().whileTrue(shooter.adjustValues());

    copilotXbox.povUp().whileTrue(climber.ascend().repeatedly()); // Climb up
    copilotXbox.povDown().whileTrue(climber.descend().repeatedly()); // Climb down

    // TRIGGERS
    // leftTOFValid.or(rightTOFValid).whileTrue(lights.set(Colors.RED,
    // Patterns.BLINK));
    // leftTOFValid.and(centerTOFValid).or((centerTOFValid).and(rightTOFValid)).whileTrue(lights.set(Colors.RED,Patterns.FAST_BLINK));
    // centerTOFValid.and((leftTOFValid.negate()).and(rightTOFValid.negate())).whileTrue(lights.set(Colors.RED,Patterns.SOLID));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public static boolean isRed() {
    try {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    } catch (NoSuchElementException e) {
      return false;
    }
  }

  public double distanceToHub() {
    Pose2d hubPose = new Pose2d(RobotContainer.isRed() ? 11.917 : 4.623, 4.030, Rotation2d.kZero);
    return hubPose.getTranslation().getDistance(getDrivebase().getPose().getTranslation());
  }

  public DoubleSupplier rotationHandler() {
    // if (copilotXbox.getHID().getLeftBumperButton())
    // return () -> copilotXbox.getRightX() * -1;
    return () -> -inhibitedRot();
  }

  public void rollingBack() {
    if (copilotXbox.getRightTriggerAxis() > 0.1) {
      rollingBackStartTime = Timer.getFPGATimestamp();
      rollingBack = false;
      return;
    }
    if (Timer.getFPGATimestamp() > rollingBackStartTime + 2) {
      rollingBack = false;
    } else {
      rollingBack = true;
    }
  }

  public void disabledRunningLights() {
    if (isRed()) {
      CommandScheduler.getInstance().schedule(
          lights.set(new NeonLights.Pattern[] { NeonLights.Pattern.FIRE_GRADIENT, NeonLights.Pattern.TRAVEL }));
    } else {
      CommandScheduler.getInstance().schedule(
          lights.set(new NeonLights.Pattern[] { NeonLights.Pattern.GREEN_BLUE_GRADIENT, NeonLights.Pattern.TRAVEL }));
    }
  }

  public void allianceLights() {
    //ALLIANCE LIGHTS
    NeonLights.Pattern hub1 = NeonLights.Pattern.OFF;
    NeonLights.Pattern hub2 = NeonLights.Pattern.OFF;
    if (DriverStation.getGameSpecificMessage().length() > 0
        && DriverStation.getGameSpecificMessage().charAt(0) == 'R') {
      hub1 = NeonLights.Pattern.GREEN_BLUE_GRADIENT;
      hub2 = NeonLights.Pattern.FIRE_GRADIENT;
    } else {
      hub2 = NeonLights.Pattern.GREEN_BLUE_GRADIENT;
      hub1 = NeonLights.Pattern.FIRE_GRADIENT;
    }
    CommandScheduler.getInstance()
        .schedule(lights.set(new NeonLights.Pattern[] { NeonLights.Pattern.PURPLE }).andThen(Commands.waitSeconds(10))
            .andThen(lights.set(new NeonLights.Pattern[] { hub1 })).andThen(Commands.waitSeconds(15))
            .andThen(lights.set(new NeonLights.Pattern[] { hub1, NeonLights.Pattern.BLINK }))
            .andThen(Commands.waitSeconds(10)).andThen(lights.set(new NeonLights.Pattern[] { hub2 }))
            .andThen(Commands.waitSeconds(15))
            .andThen(lights.set(new NeonLights.Pattern[] { hub2, NeonLights.Pattern.BLINK }))
            .andThen(Commands.waitSeconds(10)).andThen(lights.set(new NeonLights.Pattern[] { hub1 }))
            .andThen(Commands.waitSeconds(15))
            .andThen(lights.set(new NeonLights.Pattern[] { hub1, NeonLights.Pattern.BLINK }))
            .andThen(Commands.waitSeconds(10)).andThen(lights.set(new NeonLights.Pattern[] { hub2 }))
            .andThen(Commands.waitSeconds(15))
            .andThen(lights.set(new NeonLights.Pattern[] { hub2, NeonLights.Pattern.BLINK }))
            .andThen(Commands.waitSeconds(10))
            .andThen(lights.set(new NeonLights.Pattern[] { NeonLights.Pattern.PURPLE })));
  }

  public void periodic() {
    sensation.periodic();
    Logger.recordOutput("Robot/shooterRPSFromDistance", shooter.getShooterRPSFromDistance());
    Logger.recordOutput("Robot/slowMode", slowMode);
    Logger.recordOutput("Robot/finalChassisSpeeds", drivebase.driveWithAimbot());
    Logger.recordOutput("Robot/robotPose", drivebase.getPose());
    Logger.recordOutput("Robot/distanceToHub", distanceToHub());
    Logger.recordOutput("Robot/rollingBack", rollingBack);

    loaderPower = loaderPowerEntry.getDouble(0.95);
  }

  public void initializeRobotPositionBasedOnAutoRoutine() {
    Command autoroutine = getAutonomousCommand();
    if (autoroutine == null) {
      return;
    }
    String routineName = autoroutine.getName();

    if (robotPoseHasBeenSetFor.equals(routineName)) {
      return; // already set for this routine
    }

    Optional<Pose2d> startingPose = Constants.Positions.getPositionForRobot(routineName);
    if (startingPose.isEmpty()) {
      return;
    }

    drivebase.resetOdometry(startingPose.get());
    robotPoseHasBeenSetFor = routineName;
  }

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  public Conveyor getConveyor() {
    return conveyor;
  }

  public NeonLights getLights() {
    return lights;
  }

  public Climber getClimber() {
    return climber;
  }

  public Collector getCollector() {
    return collector;
  }

  public Loader getLoader() {
    return loader;
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Sensation getSensation() {
    return sensation;
  }

  public double inhibitedDriveX() {
    return slowMode ? driverXbox.getLeftX() * 0.15 : driverXbox.getLeftX();
  }

  public double inhibitedDriveY() {
    return slowMode ? driverXbox.getLeftY() * 0.15 : driverXbox.getLeftY();
  }

  public double inhibitedRot() {
    return slowMode ? driverXbox.getRightX() * 0.4 : driverXbox.getRightX();
  }
}
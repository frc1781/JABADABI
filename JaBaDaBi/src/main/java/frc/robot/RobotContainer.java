// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.Ascend;
import frc.robot.commands.swervedrive.auto.Climb;
import frc.robot.commands.swervedrive.auto.Collect;
import frc.robot.commands.swervedrive.auto.Deploy;
import frc.robot.commands.swervedrive.auto.PreShoot;
import frc.robot.commands.swervedrive.auto.Shoot;
import frc.robot.commands.swervedrive.auto.StopCollect;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import swervelib.SwerveInputStream;

public class RobotContainer {
  private String robotPoseHasBeenSetFor = "nothing";
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController copilotXbox = new CommandXboxController(1);
  private final SwerveSubsystem drivebase;
  private final Conveyor conveyor = new Conveyor();
  private final Lights lights = new Lights();
  private final Climber climber = new Climber();
  private final Collector collector = new Collector();
  private final Loader loader = new Loader();
  private final Shooter shooter = new Shooter(this);
  private final Sensation sensation = new Sensation();
  private final SendableChooser<Command> autoChooser;
  private boolean slowMode = false;

  Trigger leftTOFValid = new Trigger(() -> (sensation.isLeftTOFValid() && (sensation.getLeftTOF() < 200)));
  Trigger centerTOFValid = new Trigger(() -> (sensation.isCenterTOFValid() && (sensation.getCenterTOF() < 200)));
  Trigger rightTOFValid = new Trigger(() -> (sensation.isRightTOFValid() && (sensation.getRightTOF() < 200)));

  SwerveInputStream driveAngularVelocity;
  SwerveInputStream driveFieldOriented;

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

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("Collect", new Collect(lights, collector));
    NamedCommands.registerCommand("Climb", new Climb(climber, lights));
    NamedCommands.registerCommand("Ascend", new Ascend(climber, lights));
    NamedCommands.registerCommand("Shoot", new Shoot(loader, conveyor, shooter, lights, 4));
    NamedCommands.registerCommand("PreShoot", new PreShoot(shooter));
    NamedCommands.registerCommand("StopCollect", new StopCollect(lights, collector));
    NamedCommands.registerCommand("Deploy", new Deploy(collector));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      Logger.recordOutput("Drive/currentPose", pose);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      Logger.recordOutput("Drive/targetPose", pose);
    });

  }

  private void configureBindings() {
    Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveWithAimBot = drivebase.driveWithAimBot(driveAngularVelocity, () -> shooter.getFuelTimeOfFlight());

    drivebase.setDefaultCommand(driveFieldOriented);
    lights.setDefaultCommand(lights.set(Lights.Colors.OFF, Lights.Patterns.SOLID));
    if (Robot.getRobot() == Robots.SAVITAR) {
      loader.setDefaultCommand(loader.idle());
      shooter.setDefaultCommand(shooter.idle());
      conveyor.setDefaultCommand(conveyor.idle());
      collector.setDefaultCommand(collector.idle());
      climber.setDefaultCommand(climber.idle());
    }

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.b().onTrue(new InstantCommand(() -> slowMode = !slowMode)); // toggle slow mode
    copilotXbox.x().whileTrue(driveWithAimBot);
    driverXbox.leftBumper().whileTrue(collector.collectorAway());
    driverXbox.leftTrigger().whileTrue(collector.collectorAdjust(() -> driverXbox.getHID().getLeftTriggerAxis()));
    driverXbox.rightBumper().whileTrue(collector.collect(() -> -0.80));
    driverXbox.rightTrigger().whileTrue(collector.collect(() -> 0.80)); // intake collect
    // driverXbox.leftTrigger().whileTrue(driveWithAimBot); // drives to hub or
    // somewhere close to hub / aim

    // KEY BINDINGS (COPILOT)
    copilotXbox.leftBumper().whileTrue(shooter.shoot(() -> -100)
        .alongWith(loader.runLoader(() -> -0.85)));
    copilotXbox.leftTrigger().whileTrue(driveWithAimBot);
    copilotXbox.rightBumper().whileTrue(shooter.shoot(() -> shooter.getShooterRPSFromDistance()));
    copilotXbox.b().whileTrue(shooter.adjustHood(() -> 0.33));
    // copilotXbox.x().whileTrue(shooter.adjustHood(() -> 0.22));
    copilotXbox.a().whileTrue(shooter.subtractRPS());
    copilotXbox.x().whileTrue((shooter.shoot(() -> 90))
        .alongWith(loader.runLoader(() -> shooter.atSpeed() ? 0.85 : 0.0))
        .alongWith(conveyor.loadFuel(() -> shooter.atSpeed() ? true : false))
        .alongWith(shooter.adjustHood(() -> 0.33)));
    copilotXbox.y().whileTrue(shooter.addRPS());
    copilotXbox.rightTrigger().whileTrue((new InstantCommand(drivebase::lock))
        // .alongWith(shooter.shoot(() -> getShooterRPSFromDistance()))
        .alongWith(loader.runLoader(() -> shooter.atSpeed() ? 0.85 : 0.0))
        .alongWith(conveyor.loadFuel(() -> shooter.atSpeed() ? true : false)));
    copilotXbox.rightStick().whileTrue(
        collector.collect(() -> -0.80)
            .alongWith(loader.runLoader(() -> -0.85))
            .alongWith(conveyor.unloadFuel(() -> true))
            .alongWith(shooter.shoot(() -> -50)));

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

  public DoubleSupplier rotationHandler() {
    // if (copilotXbox.getHID().getLeftBumperButton())
    // return () -> copilotXbox.getRightX() * -1;
    return () -> -inhibitedRot();
  }

  public void disabledRunningLights() {
    if (isRed()) {
      lights.run(Lights.Colors.GREEN, Lights.Patterns.TRAVEL);
    } else {
      lights.run(Lights.Colors.BLUE, Lights.Patterns.TRAVEL);
    }
  }

  public void periodic() {
    sensation.periodic();
    Logger.recordOutput("Robot/shooterRPSFromDistance", shooter.getShooterRPSFromDistance());
    Logger.recordOutput("Robot/slowMode", slowMode);
    Logger.recordOutput("Robot/finalChassisSpeeds", drivebase.driveWithAimbot());
    Logger.recordOutput("Robot/robotPose", drivebase.getPose());
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

  public Lights getLights() {
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
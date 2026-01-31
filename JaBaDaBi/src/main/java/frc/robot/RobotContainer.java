// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.Climb;
import frc.robot.commands.swervedrive.auto.Collect;
import frc.robot.commands.swervedrive.auto.DriveToPose;
import frc.robot.commands.swervedrive.auto.Shoot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Lights.Colors;
import frc.robot.subsystems.Lights.Patterns;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.NoSuchElementException;
import java.util.Optional;

import swervelib.SwerveInputStream;

public class RobotContainer {
  private String robotPoseHasBeenSetFor = "nothing";
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // private final Sensation sensation = new Sensation();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/autoPrime"));
  // private final TankDriveTrain tankDrive = new TankDriveTrain(driverXbox);
  private final Conveyor conveyor = new Conveyor();
  private final Lights lights = new Lights();
  private final Climber climber = new Climber();
  private final Collector collector = new Collector();
  private final Hopper hopper = new Hopper();
  private final Shooter shooter = new Shooter();
  private final Sensation sensation = new Sensation();
  private final SendableChooser<Command> autoChooser;
  private double wait_seconds = 5;

  Trigger leftTOFValid = new Trigger(sensation::isLeftTOFValid);
  Trigger centerTOFValid = new Trigger(sensation::isCenterTOFValid);
  Trigger rightTOFValid = new Trigger(sensation::isRightTOFValid);


  // Driving the robot during teleOp
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8) // might be changed to 1
      .allianceRelativeControl(true)
      .cubeRotationControllerAxis(true);

  // Clone's the angular velocity input stream and converts it to a fieldRelative
  // input stream.
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> driverXbox.getRightX() * -1, () -> driverXbox.getRightY() * -1)
      .headingWhile(true);

  // Clone's the angular velocity input stream and converts it to a robotRelative
  // input stream.
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
          () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(0));

  public RobotContainer() {
    configureBindings();

    leftTOFValid.or(rightTOFValid).whileTrue(lights.set(Colors.RED, Patterns.BLINK));
    leftTOFValid.and(centerTOFValid).or((centerTOFValid).and(rightTOFValid)).whileTrue(lights.set(Colors.RED,Patterns.FAST_BLINK));
    centerTOFValid.whileTrue(lights.set(Colors.RED,Patterns.SOLID));

    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("CustomWaitCommand", new WaitCommand(SmartDashboard.getNumber("Wait Time", wait_seconds)));
    NamedCommands.registerCommand("Score", new Shoot(lights));
    NamedCommands.registerCommand("Collect", new Collect(lights));
    NamedCommands.registerCommand("Climb", new Climb(lights));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("Wait Time", wait_seconds);
  }

  private void configureBindings() {
    Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);

    drivebase.setDefaultCommand(driveFieldOriented);
    lights.setDefaultCommand(lights.set(Lights.Special.OFF));

    driverXbox.a().whileTrue(collector.collect(() -> 0));// put collect in here later
    driverXbox.b().whileTrue(collector.collect(() -> 0)); // invert floor intake here later
    driverXbox.x().whileTrue(Commands.none());
    driverXbox.y().whileTrue(Commands.none());
    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    driverXbox.rightBumper().onTrue(Commands.none());
    driverXbox.povUp().whileTrue(new Climb(lights)); //Climb up
    driverXbox.povDown().whileTrue(new Climb(lights)); //Climb down
    driverXbox.leftTrigger().whileTrue(new DriveToPose(lights)); // drives to hub or somewhere close to hub
    driverXbox.rightTrigger().whileTrue(new Shoot(lights)); // Shoot

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

  public void disabledRunningLights() {
    if (isRed()) {
      lights.run(Lights.Colors.GREEN, Lights.Patterns.TRAVEL);
    } else {
      lights.run(Lights.Colors.BLUE, Lights.Patterns.TRAVEL);
    }
  }

  public void periodic() {
    sensation.periodic();
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
}

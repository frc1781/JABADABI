// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Constants;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;

// public class TankDriveTrain extends SubsystemBase {
//   TalonFX left1;
//   TalonFX left2;

//   TalonFX right1;
//   TalonFX right2;

//   DifferentialDrive drive;
//   private final CommandXboxController controller;

//   /** Creates a new TankDriveTrain. */
//   public TankDriveTrain(CommandXboxController driverXbox) {
//     left1 = new TalonFX(Constants.TankDrivebaseConstants.LEFT_1);
//     left2 = new TalonFX(Constants.TankDrivebaseConstants.LEFT_2);
//     right1 = new TalonFX(Constants.TankDrivebaseConstants.RIGHT_1);
//     right2 = new TalonFX(Constants.TankDrivebaseConstants.RIGHT_2);
//     configureMotors();
    
//     this.controller = driverXbox;
    
//     drive = new DifferentialDrive(left1::set, right1::set);
//   }
  
//   public void configureMotors() {
//     TalonFXConfiguration leftConfig = new TalonFXConfiguration();
//     TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    
//     // Not inverted and brake
//     leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//     // Inverted and brake
//     rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

//     left1.getConfigurator().apply(leftConfig);
//     left2.getConfigurator().apply(leftConfig);
//     right1.getConfigurator().apply(rightConfig);
//     right2.getConfigurator().apply(rightConfig);

//     left2.setControl(new Follower(left1.getDeviceID(), MotorAlignmentValue.Opposed));
//     right2.setControl(new Follower(right1.getDeviceID(), MotorAlignmentValue.Opposed));
//   }

//   @Override
//   public void periodic() {
//     if (controller.getLeftY() >= 0.1 || controller.getLeftY() <= -0.1 || controller.getRightX() >= 0.1 || controller.getRightX() <= -0.1) {
//       drive.arcadeDrive(-controller.getLeftY(), controller.getRightX());
//     }
//     else {
//       drive.arcadeDrive(0, 0);
//     }
//   }
// }

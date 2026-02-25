package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import CRA.PIDTuning;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot; 
import edu.wpi.first.wpilibj.XboxController;

public class Shooter extends SubsystemBase {

    private RobotContainer robotContainer;

    private TalonFX leftShooter;
    private TalonFX rightShooter;
    private Servo hoodServoLeft;
    private Servo hoodServoRight;

    private TalonFXConfiguration leftShooterConfig;
    private TalonFXConfiguration rightShooterConfig;
    private SparkFlexConfig motorHoodConfig;

    private Slot0Configs leftShooterProfile;
    private Slot0Configs rightShooterProfile;
    private SparkClosedLoopController motorHoodController;

    private double fuelTimeOfFlight = 0.0;
    private double hoodPosition;

    private final VelocityVoltage leftVelocityReq = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage rightVelocityReq = new VelocityVoltage(0).withSlot(0);

    public Shooter(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        // leftShooter = new TalonFX(Constants.Shooter.SHOOTER_1_CAN_ID);
        // rightShooter = new TalonFX(Constants.Shooter.SHOOTER_2_CAN_ID);
        hoodServoLeft = new Servo(2); 
        // hoodServoRight = new Servo(3); 
        hoodPosition = 0.5;

        // leftShooterProfile = new Slot0Configs() // IDK YET EITHER
        //         .withKS(Constants.Shooter.S)
        //         .withKV(Constants.Shooter.V)
        //         .withKA(Constants.Shooter.A)
        //         .withKP(Constants.Shooter.P);

        // rightShooterProfile = new Slot0Configs() // IDK YET EITHER
        //         .withKS(Constants.Shooter.S)
        //         .withKV(Constants.Shooter.V)
        //         .withKA(Constants.Shooter.A)
        //         .withKP(Constants.Shooter.P);

        // leftShooterConfig = new TalonFXConfiguration()
        //         .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80))
        //         .withMotorOutput(new MotorOutputConfigs()
        //                 .withNeutralMode(NeutralModeValue.Coast)
        //                 .withInverted(InvertedValue.CounterClockwise_Positive))
        //         .withSlot0(leftShooterProfile);

        // rightShooterConfig = new TalonFXConfiguration()
        //         .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80))
        //         .withMotorOutput(new MotorOutputConfigs()
        //                 .withNeutralMode(NeutralModeValue.Coast)
        //                 .withInverted(InvertedValue.Clockwise_Positive))
        //         .withSlot0(rightShooterProfile);

        // leftShooter.getConfigurator().apply(leftShooterConfig);
        // rightShooter.getConfigurator().apply(rightShooterConfig);

        hoodServoLeft.setBoundsMicroseconds(2100, 1800, 1500, 1200, 900);
        // hoodServoRight.setBoundsMicroseconds(2100, 1800, 1500, 1200, 900);
    }

    @Override
    public void periodic() {
        // leftShooter.setControl(leftVelocityReq);
        // rightShooter.setControl(rightVelocityReq);

        // Logger.recordOutput("Shooter/leftVoltage", leftShooter.getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("Shooter/leftCurrent", leftShooter.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/hoodPosition", hoodPosition);
        // Logger.recordOutput("Shooter/rightVoltage", rightShooter.getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("Shooter/rightCurrent", rightShooter.getSupplyCurrent().getValueAsDouble());
    }

    public Command idle() {
        return new RunCommand(() -> {
            // leftVelocityReq.withVelocity(0);
            // rightVelocityReq.withVelocity(0);
           hoodPosition = 0.5;
        //  hoodServoLeft.set(hoodPosition); 
        // hoodServoRight.set(hoodPosition);
        }, this);  
    }

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            // leftVelocityReq.withVelocity(setPoint.getAsDouble());
            // rightVelocityReq.withVelocity(setPoint.getAsDouble());
          //  hoodServoLeft.set(hoodPosition);
         //   hoodServoRight.set(hoodPosition);
        }, this);
    }

    public Command shoot() {
        return new RunCommand(() -> {
            double setPoint = getShooterRPMFromDistance();
            hoodPosition = getHoodPositionFromDistance();
            // leftVelocityReq.withVelocity(setPoint);
            // rightVelocityReq.withVelocity(setPoint);
          //  hoodServoLeft.set(hoodPosition);
           // hoodServoRight.set(hoodPosition);
        }, this);
    }

    public Command adjustHood(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            hoodPosition = setPoint.getAsDouble();
        //  hoodServoLeft.set(hoodPosition);
        // hoodServoRight.set(hoodPosition);
        }, this);
    }

    public double getFuelTimeOfFlight() {
        return fuelTimeOfFlight; // placeholder for actual time of flight sensor value, will need to be updated with actual sensor reading
    }

    public double getShooterRPMFromDistance() {
        Pose2d hubPose = new Pose2d(RobotContainer.isRed() ? 11.917 : 4.623, 4.030, Rotation2d.kZero); // FROM PATHHUBCALCULATIONS
        double distanceToHub = hubPose.getTranslation().getDistance(robotContainer.getDrivebase().getPose().getTranslation());
        return distanceToHub * 350; //purely fictional, will need to be converted to RPM using a formula that we will determine through testing
    }

    public double getHoodPositionFromDistance() {
        Pose2d hubPose = new Pose2d(RobotContainer.isRed() ? 11.917 : 4.623, 4.030, Rotation2d.kZero); // FROM PATHHUBCALCULATIONS
        double distanceToHub = hubPose.getTranslation().getDistance(robotContainer.getDrivebase().getPose().getTranslation());
        return 0.5; //  distanceToHub * ?????; //currently just returns distanceToHub, will need to be converted to RPM using a formula that we will determine through testing
    }
}

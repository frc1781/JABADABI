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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

    private final VelocityVoltage leftVelocityReq;
    private final VelocityVoltage rightVelocityReq;
    private double fuelTimeOfFlight = 0.0;
    private double hoodPosition;

    private double leftRPS;
    private double rightRPS;

    private boolean alreadySetRPS;

    private double leftReqRPS;
    private double rightReqRPS;

    private GenericEntry kVElastic;
    private GenericEntry kPElastic;

    private ShuffleboardTab shooterTab;

    public Shooter(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        // leftShooter = new TalonFX(Constants.Shooter.SHOOTER_1_CAN_ID);
        // rightShooter = new TalonFX(Constants.Shooter.SHOOTER_2_CAN_ID);
        hoodServoLeft = new Servo(Constants.Shooter.HOOD_LEFT_PWM);
        hoodServoRight = new Servo(Constants.Shooter.HOOD_RIGHT_PWM);
        hoodPosition = 0.5;

        leftVelocityReq = new VelocityVoltage(0).withSlot(0);
        rightVelocityReq = new VelocityVoltage(0).withSlot(0);

        shooterTab = Shuffleboard.getTab(getName());
        kVElastic = shooterTab.add(getName() + "kV", Constants.Shooter.V).getEntry();
        kPElastic = shooterTab.add(getName() + "kP", Constants.Shooter.P).getEntry();

        leftShooterProfile = new Slot0Configs()
                .withKV(Constants.Shooter.V)
                .withKP(Constants.Shooter.P);

        rightShooterProfile = new Slot0Configs()
                .withKV(Constants.Shooter.V)
                .withKP(Constants.Shooter.P);

        leftShooterConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(leftShooterProfile);

        rightShooterConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withSlot0(rightShooterProfile);

        leftShooter.getConfigurator().apply(leftShooterConfig);
        rightShooter.getConfigurator().apply(rightShooterConfig);

        hoodServoLeft.setBoundsMicroseconds(2100, 1800, 1500, 1200, 900);
        hoodServoRight.setBoundsMicroseconds(2100, 1800, 1500, 1200, 900);
    }

    @Override
    public void periodic() {

        leftVelocityReq.withVelocity(leftReqRPS);
        rightVelocityReq.withVelocity(rightReqRPS);

        leftShooter.setControl(leftVelocityReq);
        rightShooter.setControl(rightVelocityReq);

        leftRPS = leftVelocityReq.Velocity;
        rightRPS = rightVelocityReq.Velocity;

        hoodServoLeft.set(hoodPosition);
        hoodServoRight.set(hoodPosition);

        Logger.recordOutput("Shooter/leftVoltage", leftShooter.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/leftCurrent", leftShooter.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput("Shooter/rightVoltage", rightShooter.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/rightCurrent", rightShooter.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput("Shooter/leftVelocity", leftShooter.getVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter/rightVelocity", rightShooter.getVelocity().getValueAsDouble());

        Logger.recordOutput("Shooter/leftReqVelocity", leftVelocityReq.Velocity);
        Logger.recordOutput("Shooter/rightReqVelocity", rightVelocityReq.Velocity);

        Logger.recordOutput("Shooter/leftHoodRequestedPosition", hoodServoLeft.get());
        Logger.recordOutput("Shooter/rightHoodRequestedPosition", hoodServoRight.get());

        Logger.recordOutput("Shooter/shooterRPMFromDistance", getShooterRPSFromDistance());
        Logger.recordOutput("Shooter/hoodPositionFromDistance", getHoodPositionFromDistance());
    }

    public Command idle() {
        return new RunCommand(() -> {
            leftReqRPS = 0;
            rightReqRPS = 0;
            alreadySetRPS = false;
            hoodPosition = 0.5;
        }, this);

    }

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            if (!alreadySetRPS) {
                leftReqRPS = setPoint.getAsDouble();
                rightReqRPS = setPoint.getAsDouble();
            }
            alreadySetRPS = true;
            hoodServoLeft.set(hoodPosition);
            hoodServoRight.set(hoodPosition);
        }, this);
    }

    public Command shoot() {
        return new RunCommand(() -> {
            leftReqRPS = getShooterRPSFromDistance();
            rightReqRPS = getShooterRPSFromDistance();
        }, this);
    }

    public Command addRPS() {
        return new InstantCommand(() -> {
            leftReqRPS += 5;
            rightReqRPS += 5;
        });
    }

    public Command subtractRPS() {
        return new InstantCommand(() -> {
            leftReqRPS -= 5;
            rightReqRPS -= 5;
        });
    }

    public Command adjustHood() {
        return new RunCommand(() -> {
            hoodPosition = getHoodPositionFromDistance();
        }, this);
    }

    public Command adjustHood(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            hoodPosition = setPoint.getAsDouble();
        }, this);
    }

    public Command adjustValues() {
        return new InstantCommand(() -> {
            leftShooterProfile = new Slot0Configs() // IDK YET EITHER
                    .withKV(kVElastic.getDouble(0))
                    .withKP(kPElastic.getDouble(0));

            rightShooterProfile = new Slot0Configs() // IDK YET EITHER
                    .withKV(kVElastic.getDouble(0))
                    .withKP(kPElastic.getDouble(0));

            leftShooterConfig = new TalonFXConfiguration()
                    .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80))
                    .withMotorOutput(new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Coast)
                            .withInverted(InvertedValue.CounterClockwise_Positive))
                    .withSlot0(leftShooterProfile);

            rightShooterConfig = new TalonFXConfiguration()
                    .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80))
                    .withMotorOutput(new MotorOutputConfigs()
                            .withInverted(InvertedValue.Clockwise_Positive))
                    .withSlot0(rightShooterProfile);

            leftShooter.getConfigurator().apply(leftShooterConfig);
            rightShooter.getConfigurator().apply(rightShooterConfig);
        }, this);
    }

    public double getFuelTimeOfFlight() {
        Pose2d hubPose = new Pose2d(RobotContainer.isRed() ? 11.917 : 4.623, 4.030, Rotation2d.kZero); // FROM
                                                                                                       // PATHHUBCALCULATIONS
        double distanceToHub = hubPose.getTranslation()
                .getDistance(robotContainer.getDrivebase().getPose().getTranslation());
        return distanceToHub; // placeholder for actual time of flight sensor value, will need to be updated
                              // with actual sensor reading
    }

    public double getShooterRPSFromDistance() {
        Pose2d hubPose = new Pose2d(RobotContainer.isRed() ? 11.917 : 4.623, 4.030, Rotation2d.kZero); // FROM
                                                                                                       // PATHHUBCALCULATIONS
        double distanceToHub = hubPose.getTranslation()
                .getDistance(robotContainer.getDrivebase().getPose().getTranslation());
        return distanceToHub; // currently just returns distanceToHub, will need to be converted to RPM using
                              // a formula that we will determine through testing
    }

    public double getHoodPositionFromDistance() {
        Pose2d hubPose = new Pose2d(RobotContainer.isRed() ? 11.917 : 4.623, 4.030, Rotation2d.kZero); // FROM PATHHUBCALCULATIONS
        double distanceToHub = hubPose.getTranslation().getDistance(robotContainer.getDrivebase().getPose().getTranslation());
        return 0.5; //  distanceToHub * ?????; //currently just returns distanceToHub, will need to be converted to RPM using a formula that we will determine through testing
    }
}

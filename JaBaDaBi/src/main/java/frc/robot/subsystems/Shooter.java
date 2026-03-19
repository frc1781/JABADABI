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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.EEUtil;
import CRA.PIDTuning;

public class Shooter extends SubsystemBase {

    private RobotContainer robotContainer;

    private TalonFX leftShooter;
    private TalonFX rightShooter;
    // private SparkFlex motorHood;

    private TalonFXConfiguration leftShooterConfig;
    private TalonFXConfiguration rightShooterConfig;
    private SparkFlexConfig motorHoodConfig;

    private Slot0Configs leftShooterProfile;
    private Slot0Configs rightShooterProfile;

    private final VelocityVoltage leftVelocityReq;
    private final VelocityVoltage rightVelocityReq;

    private Servo hoodServos;

    private boolean alreadySetRPS;

    public boolean reachedSpeed;

    private double leftReqRPS;
    private double rightReqRPS;

    private GenericEntry kVElastic;
    private GenericEntry kPElastic;

    private GenericEntry RPSSlopeEntry;
    private GenericEntry RPSInterceptEntry;

    private ShuffleboardTab shooterTab;
    private double hoodPosition;

    private double RPSSlope = 6.5;
    private double RPSIntercept = 40;

    public Shooter(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        reachedSpeed = false;
        leftShooter = new TalonFX(Constants.Shooter.SHOOTER_1_CAN_ID);
        rightShooter = new TalonFX(Constants.Shooter.SHOOTER_2_CAN_ID);

        leftVelocityReq = new VelocityVoltage(0).withSlot(0);
        rightVelocityReq = new VelocityVoltage(0).withSlot(0);

        shooterTab = Shuffleboard.getTab(getName());
        kVElastic = shooterTab.add(getName() + "kV", Constants.Shooter.V).getEntry();
        kPElastic = shooterTab.add(getName() + "kP", Constants.Shooter.P).getEntry();
        RPSSlopeEntry = shooterTab.add(getName() + "RPSSlope", RPSSlope).getEntry();
        RPSInterceptEntry = shooterTab.add(getName() + "RPSIntercept", RPSIntercept).getEntry();

        hoodServos = new Servo(Constants.Shooter.HOOD_PWM);
        hoodPosition = 1;

        leftShooterProfile = new Slot0Configs()
                .withKV(Constants.Shooter.V)
                .withKP(Constants.Shooter.P);

        rightShooterProfile = new Slot0Configs()
                .withKV(Constants.Shooter.V)
                .withKP(Constants.Shooter.P);

        leftShooterConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(30))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(leftShooterProfile);

        rightShooterConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(30))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withSlot0(rightShooterProfile);

        leftShooter.getConfigurator().apply(leftShooterConfig);
        rightShooter.getConfigurator().apply(rightShooterConfig);

        hoodServos.setBoundsMicroseconds(2500, 2500, 1500, 700, 500);
        Logger.recordOutput(getName() + "/currentCommand", "notStarted");
    }

    @Override
    public void periodic() {

        leftVelocityReq.withVelocity(leftReqRPS);
        rightVelocityReq.withVelocity(rightReqRPS);

        leftShooter.setControl(leftVelocityReq);
        rightShooter.setControl(rightVelocityReq);

        hoodPosition = EEUtil.clamp(0.40, 1.0, hoodPosition);
        hoodServos.set(hoodPosition);

        RPSSlope = RPSSlopeEntry.getDouble(6.5);
        RPSIntercept = RPSInterceptEntry.getDouble(40);

        Logger.recordOutput("Shooter/leftVoltage", leftShooter.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/leftCurrent", leftShooter.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/rightVoltage", rightShooter.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/rightCurrent", rightShooter.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/leftVelocity", leftShooter.getVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter/rightVelocity", rightShooter.getVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter/leftReqVelocity", leftVelocityReq.Velocity);
        Logger.recordOutput("Shooter/rightReqVelocity", rightVelocityReq.Velocity);
        Logger.recordOutput("Shooter/RPSSlope", RPSSlope);
        Logger.recordOutput("Shooter/RPSIntercept", RPSIntercept);
        Logger.recordOutput("Shooter/reachedSpeed", reachedSpeed);
        Logger.recordOutput("Shooter/hoodPosition", hoodPosition);
    }

    public Command idle() {
        return new InstantCommand(() -> {
            leftReqRPS = 0;
            rightReqRPS = 0;
            alreadySetRPS = false;
            reachedSpeed = false;
            Logger.recordOutput("Shooter/currentCommand", "idle");
            Logger.recordOutput("Shooter/RPSAdjustments", "idle");
        }, this);
    }

    public boolean atSpeed() {
        if (reachedSpeed) {
            return true;
        }
        reachedSpeed = leftShooter.getVelocity().getValueAsDouble() > leftVelocityReq.Velocity - 3 &&
                leftVelocityReq.Velocity > 10;
        return reachedSpeed;
    }

    public boolean atZero() {
        if (leftVelocityReq.Velocity <= 10 && rightVelocityReq.Velocity <= 10) {
            Logger.recordOutput("Shooter/atZero", leftShooter.getVelocity().getValueAsDouble() <= 10
                    && rightShooter.getVelocity().getValueAsDouble() <= 10);
            return leftShooter.getVelocity().getValueAsDouble() <= 10
                    && rightShooter.getVelocity().getValueAsDouble() <= 10;
        }
        return false;
    }

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            Logger.recordOutput("Shooter/currentCommand", "shoot");
            if (!alreadySetRPS || setPoint.getAsDouble() != leftReqRPS || setPoint.getAsDouble() != rightReqRPS) {
                leftReqRPS = setPoint.getAsDouble();
                rightReqRPS = setPoint.getAsDouble();
            }
            hoodServos.set(hoodPosition);
            alreadySetRPS = true;
        }, this);
    }

    public Command shooterToSpeed(DoubleSupplier rps) {
        return new FunctionalCommand(
                () -> {
                    leftReqRPS = rps.getAsDouble();
                    rightReqRPS = rps.getAsDouble();
                    Logger.recordOutput(getName() + "/currentCommand", "shooterToSpeed");
                },
                () -> {

                },
                (interrupted) -> {
                    Logger.recordOutput(getName() + "/currentCommand", "shooterAtSpeed");
                },
                () -> atSpeed(),
                this);
    };

    public Command shoot() {
        return new RunCommand(() -> {
            Logger.recordOutput("Shooter/currentCommand", "shootUsingCalculatedRPS");
            double setPoint = getShooterRPSFromDistance();
            leftReqRPS = setPoint;
            rightReqRPS = setPoint;
            hoodServos.set(hoodPosition);
        }, this);
    }

    public Command addRPS() {
        return new InstantCommand(() -> {
            Logger.recordOutput("Shooter/RPSAdjustments", "addRPS");
            leftReqRPS += 2;
            rightReqRPS += 2;
        });
    }

    public Command subtractRPS() {
        return new InstantCommand(() -> {
            Logger.recordOutput("Shooter/RPSAdjustments", "subtractRPS");
            leftReqRPS -= 2;
            rightReqRPS -= 2;
        });
    }

    public Command adjustHood(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            Logger.recordOutput("Shooter/currentHoodCommand", "adjustingHood");
            hoodPosition = setPoint.getAsDouble();
            hoodServos.set(hoodPosition);
        });
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
                            .withNeutralMode(NeutralModeValue.Coast)
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
        if (robotContainer.distanceToHub() <= 1.5) {
            return 47.5;
        }
        return 6.5 * robotContainer.distanceToHub() + 40;
    }

    public double getHoodPositionFromDistance() {
        if (robotContainer.distanceToHub() <= 1.5) {
            return 0.6;
        }
        return 1;
    }
}

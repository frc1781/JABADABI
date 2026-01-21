package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
private SparkFlex leftshooter;
private SparkFlex rightshooter; 

private SparkFlexConfig leftshooterConfig;
private SparkFlexConfig rightshooterConfig;





public Shooter() {
    leftshooter= new SparkFlex(40, MotorType.kBrushless);
    rightshooter= new SparkFlex(41, MotorType.kBrushless);

    leftshooterConfig= new SparkFlexConfig();
    leftshooterConfig.idleMode(IdleMode.kCoast);
    leftshooterConfig.smartCurrentLimit(40);
    leftshooterConfig.inverted(false);
    rightshooterConfig= new SparkFlexConfig();
    rightshooterConfig.follow(40, true);

    leftshooter.configure(leftshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightshooter.configure(rightshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    



} 






}

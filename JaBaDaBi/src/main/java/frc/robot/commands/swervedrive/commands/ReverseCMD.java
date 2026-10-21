package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class ReverseCMD extends ParallelCommandGroup{
Loader loader;
Shooter shooter;

public ReverseCMD(Loader loader, Shooter shooter){
this.loader = loader;
this.shooter = shooter;
//addRequirements(loader, shooter);

addCommands(
loader.runLoader(() -> 0.95).withTimeout(1.0));
shooter.shoot(() -> 60).withTimeout(2.0);
}
}

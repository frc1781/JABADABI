package frc.robot.commands.swervedrive.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Deploy;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class AutoShootCustom extends SequentialCommandGroup {
    
    public AutoShootCustom(Loader loader, Conveyor conveyor, Shooter shooter, Deploy deploy, Intake intake, DoubleSupplier speed) {
        addRequirements(deploy, intake, loader, conveyor, shooter);
        addCommands(
            new ParallelCommandGroup(
                shooter.shoot(speed),
                loader.runLoader(() -> shooter.atSpeed() ? 0.95 : 0.0),
                conveyor.convey(() -> shooter.atSpeed() ? 1 : 0),
                new AgitateFuel(deploy, intake)
            )
        );
    }
}

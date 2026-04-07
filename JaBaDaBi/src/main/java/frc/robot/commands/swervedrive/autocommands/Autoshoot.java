package frc.robot.commands.swervedrive.autocommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Deploy;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;

public class Autoshoot extends SequentialCommandGroup {

    public Autoshoot(Loader loader, Conveyor conveyor, Shooter shooter, Deploy collector, DoubleSupplier speed) {
        addRequirements(collector, loader, conveyor, shooter);
        addCommands(
            new ParallelCommandGroup(
                //shooter.shoot(speed),
                loader.runLoader(() -> shooter.atSpeed() ? 0.95 : 0.0),
                conveyor.convey(() -> shooter.atSpeed() ? 1 : 0),
                collector.agitateFuel()
            )
        );
    }

}

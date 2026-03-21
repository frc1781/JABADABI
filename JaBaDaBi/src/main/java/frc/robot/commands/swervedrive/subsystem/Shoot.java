package frc.robot.commands.swervedrive.subsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;

public class Shoot extends SequentialCommandGroup {

    public Shoot(Loader loader, Conveyor conveyor, Shooter shooter, Collector collector, double speed) {
        addRequirements(collector, loader, conveyor, shooter);
        addCommands(
            new ParallelCommandGroup(
                shooter.shoot(() -> speed),
                //loader.runLoader(() -> shooter.reachedSpeed() ? 0.95 : 0.0),
               // conveyor.convey(() -> shooter.reachedSpeed() ? 1 : 0),
                loader.runLoader(() -> shooter.atSpeed() ? 0.95 : 0.0),
                conveyor.convey(() -> shooter.atSpeed() ? 1 : 0),
                collector.agitateFuel()
            )
        );
    }

}

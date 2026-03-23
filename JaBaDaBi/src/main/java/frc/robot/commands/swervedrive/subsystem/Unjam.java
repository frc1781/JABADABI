package frc.robot.commands.swervedrive.subsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector;

public class Unjam extends SequentialCommandGroup {
    public Unjam(Collector collector, Loader loader, Conveyor conveyor, Shooter shooter) {
        addRequirements(collector, loader, conveyor, shooter);
        addCommands(
                collector.deploy(() -> collector.HALF_WAY_SET_POINT),
                new ParallelCommandGroup(
                        collector.deploy(() -> 0.33),
                        loader.runLoader(() -> -0.7),
                        conveyor.unloadFuel(() -> true)),
                        shooter.shoot(() -> -50)
                );
    }
}

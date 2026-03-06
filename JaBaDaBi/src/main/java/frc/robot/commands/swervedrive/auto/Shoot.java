package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Loader;

public class Shoot extends SequentialCommandGroup {

    public Shoot(Loader loader, Conveyor conveyor, Shooter shooter,Lights lights, double duration) {
        addRequirements(loader, conveyor, shooter, lights);
        addCommands(
                new ParallelRaceGroup(
                        shooter.shoot(() -> 55),
                        new WaitUntilCommand(() -> shooter.atSpeed()),
                        new InstantCommand(() -> lights.run(Lights.Colors.RED, Lights.Patterns.SOLID))),
                new ParallelCommandGroup(
                        shooter.shoot(() -> 55),
                        loader.runLoader(() -> 1),
                        conveyor.loadFuel(() -> true)).withTimeout(duration),
                        shooter.shoot(() -> 0),
                        new InstantCommand((() -> lights.run(Lights.Colors.GREEN, Lights.Patterns.BLINK)))
                );
    }

}

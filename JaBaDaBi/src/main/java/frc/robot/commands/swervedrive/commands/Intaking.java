package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class Intaking extends SequentialCommandGroup {
    Deploy deploy;
    Loader loader;
    Conveyor conveyor;

    public Intaking(Deploy deploy, Loader loader, Conveyor conveyor) {
        this.deploy = deploy;
        this.loader = loader;
        this.conveyor = conveyor;
        addRequirements(deploy, loader, conveyor);
        addCommands(
            new ParallelCommandGroup(
                deploy.deploy(), 
                loader.runLoader(() -> -0.8),
                conveyor.convey(() -> 1)
            )
        );
    }
}

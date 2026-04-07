package frc.robot.commands.swervedrive.autocommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Deploy;

public class Deploying extends SequentialCommandGroup {
    Deploy deploy;
    Loader loader;

    public Deploying(Deploy deploy, Loader loader) {
        this.deploy = deploy;
        this.loader = loader;
        addRequirements(deploy, loader);
        addCommands(
            new ParallelCommandGroup(
            deploy.deploy(() -> deploy.COLLECT_SET_POINT),
            loader.runLoader(() -> -0.8))
        );
    }
}

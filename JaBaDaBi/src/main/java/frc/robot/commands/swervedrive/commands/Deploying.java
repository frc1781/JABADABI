package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Deploy;
import frc.robot.Constants;

public class Deploying extends ParallelCommandGroup {
    Deploy deploy;
    Loader loader;

    public Deploying(Deploy deploy, Loader loader) {
        this.deploy = deploy;
        this.loader = loader;
        //addRequirements(deploy, loader);
        addCommands(
            deploy.setDeploy(() -> Constants.Deploy.COLLECT_SET_POINT),
            loader.runLoader(() -> -0.8)
        );
    }
}

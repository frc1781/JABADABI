package frc.robot.commands.swervedrive.commands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class Intaking extends ParallelCommandGroup {
    Deploy deploy;
    Loader loader;
    Conveyor conveyor;

    public Intaking(Deploy deploy, Loader loader, Intake intake, Conveyor conveyor) {
        this.deploy = deploy;
        this.loader = loader;
        this.conveyor = conveyor;
        //addRequirements(deploy, loader, conveyor);
        addCommands(
                deploy.setDeploy(() -> Constants.Deploy.COLLECT_SET_POINT),
                intake.intake(), 
                loader.runLoader(() -> -0.8),
                conveyor.convey(() -> 1)
        );
    }
}

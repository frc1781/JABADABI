package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class PreShoot extends SequentialCommandGroup {

    Shooter shooter;

    public PreShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        addCommands(
            shooter.shoot(() -> 3000)
        );
    }

}

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.Colors;
import frc.robot.subsystems.Lights.Patterns;

public class Collect extends Command {
    Lights lights;

    public Collect(Lights lights) {
        this.lights = lights;
    }

    @Override
    public void initialize() {
        System.out.println(getName() + "started");
    }

    @Override
    public void execute() {
        lights.run(Colors.YELLOW, Patterns.MARCH);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println(getName() + "interrupted");
        } else {
            System.out.println(getName() + "ended");
        }
    }
}

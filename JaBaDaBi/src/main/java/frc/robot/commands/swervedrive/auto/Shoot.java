package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.Colors;
import frc.robot.subsystems.Lights.Patterns;

public class Shoot extends Command {
    Timer timer;
    Lights lights;

    public Shoot(Lights lights) {
        this.lights = lights;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        System.out.println(getName() + "started");
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        lights.run(Colors.GREEN, Patterns.BLINK);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 5;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println(getName() + "interrupted");
        } else {
            System.out.println(getName() + "ended");
        }
        timer.stop();
        timer.reset();
    }
}

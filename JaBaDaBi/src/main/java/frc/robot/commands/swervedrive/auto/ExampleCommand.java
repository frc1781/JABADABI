package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeonLights;

public class ExampleCommand extends Command {
    Timer timer;
    NeonLights lights;

    public ExampleCommand(NeonLights lights) {
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
        lights.set(new NeonLights.Pattern[]{NeonLights.Pattern.GREEN, NeonLights.Pattern.BLINK});
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

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

public class Sensation
{
    //NOTE: DIGITAL INPUT RETURNS TRUE FOR BEAM NOT BROKEN AND FALSE FOR BROKEN BEAM
    //      SO IT REPORTING NEGATED VALUE FOR DETECTION OF SOMETHING 
    DigitalInput enterBeam;
    DigitalInput hopperBackBeam;
    DigitalInput hopperFrontBeam;
    DigitalInput exitBeam;

    public Sensation()
    {
        enterBeam = new DigitalInput(Constants.SensationConstants.enter);
        hopperBackBeam = new DigitalInput(Constants.SensationConstants.hopperBack);
        hopperFrontBeam = new DigitalInput(Constants.SensationConstants.hopperFront);
        exitBeam = new DigitalInput(Constants.SensationConstants.exit);
    }

    public boolean coralPresent() {
        return !enterBeam.get() || !hopperBackBeam.get() || !hopperFrontBeam.get();
    }

    public boolean coralInHopper() {
        return !hopperBackBeam.get() || !hopperFrontBeam.get();
    }

    public boolean coralExitedHopper() {
        return !exitBeam.get() && hopperBackBeam.get() && hopperFrontBeam.get();
    }
}
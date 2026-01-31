package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.EEtimeOfFlight;
import org.littletonrobotics.junction.Logger;

public class Sensation
{
    private EEtimeOfFlight leftTOF;
    private EEtimeOfFlight centerTOF;
    private EEtimeOfFlight rightTOF;

    public Sensation()
    {
        leftTOF = new EEtimeOfFlight(Constants.Sensation.LEFT_TOF, 20);
        centerTOF = new EEtimeOfFlight(Constants.Sensation.CENTER_TOF, 20);
        rightTOF = new EEtimeOfFlight(Constants.Sensation.RIGHT_TOF, 20);
    }

    public void periodic() {
        Logger.recordOutput("Sensation/leftTOFRange", getLeftTOF());
        Logger.recordOutput("Sensation/centerTOFRange", getCenterTOF());
        Logger.recordOutput("Sensation/rightTOFRange", getRightTOF());
        
        Logger.recordOutput("Sensation/leftTOFValid", isLeftTOFValid());
        Logger.recordOutput("Sensation/centerTOFValid", isCenterTOFValid());
        Logger.recordOutput("Sensation/rightTOFValid", isRightTOFValid());
    }

    public double getLeftTOF() {
        return leftTOF.getRange();
    }
    public double getCenterTOF() {
        return centerTOF.getRange();
    }
    public double getRightTOF() {
        return rightTOF.getRange();
    }

    public boolean isLeftTOFValid() {
        return leftTOF.isRangeValidRegularCheck();
    }
    public boolean isCenterTOFValid() {
        return centerTOF.isRangeValidRegularCheck();
    }
    public boolean isRightTOFValid() {
        return rightTOF.isRangeValidRegularCheck();
    }

    
}
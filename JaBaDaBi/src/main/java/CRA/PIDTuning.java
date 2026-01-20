package CRA;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
/**
 * PIDTuning class for real-time PID controller tuning on the Shuffleboard.
 * 
 * This class allows for live adjustment of PID controller parameters (kP, kI, kD, kF)
 * through the Shuffleboard interface, enabling quick tuning during robot operation.
 * Check in CRA/ExampleMechanism.java for an example implmenetation.
 * 
 * Usage:
 * - Create an instance of PIDTuning with a unique mechanism name and initial PID values.
 * - Use `getPID()` to retrieve the current PID values for use in your control logic.
 */
public class PIDTuning {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private String mechanism;
    private ShuffleboardTab tab;

    private GenericEntry kPEntry;
    private GenericEntry kIEntry;
    private GenericEntry kDEntry;
    private GenericEntry kFEntry;

    /**
     * Creates a new PIDTuning instance for real-time PID adjustment.
     * 
     * This constructor initializes a PID tuning interface for a specific mechanism,
     * setting up NetworkTables entries and starting values for live tuning during
     * robot testing.
     * 
     * @param mechanism_name Unique identifier for the mechanism being tuned (e.g., "shooter", "arm").
     *                      This name is used to create distinct NetworkTables entries and 
     *                      organize tuning parameters on the shuffleboard.
     * @param starting_kP Initial proportional gain value. Controls how aggressively the 
     *                   controller responds to current error.
     * @param starting_kI Initial integral gain value. Helps eliminate steady-state error
     *                   by accumulating error over time.
     * @param starting_kD Initial derivative gain value. Reduces overshoot by responding
     *                   to the rate of change of error.
     * @param starting_kF Initial feedforward gain value. Provides open-loop control
     *                   to help the system reach the desired setpoint faster.
     */
    public PIDTuning(String mechanism_name, double starting_kP, double starting_kI, double starting_kD, double starting_kF) {
        this.kP = starting_kP;
        this.kI = starting_kI;
        this.kD = starting_kD;
        this.kF = starting_kF;
        this.mechanism = mechanism_name;

        tab = Shuffleboard.getTab(mechanism); // Creating the shuffleboard tab
        onDashboard(); // Initialize the entries on the Shuffleboard tab
    }

    /**
     * Initializes the PID tuning entries on the Shuffleboard dashboard. Runs internally.
     * 
     * This method creates and adds entries for kP, kI, kD, and kF to the specified
     * Shuffleboard tab, allowing for real-time adjustment of these parameters.
     */
    public void onDashboard() {
        kPEntry = tab.add(mechanism + " kP", kP).getEntry();
        kIEntry = tab.add(mechanism + " kI", kI).getEntry();
        kDEntry = tab.add(mechanism + " kD", kD).getEntry();
        kFEntry = tab.add(mechanism + " kF", kF).getEntry();
    }
    
    /**
     * Retrieves the current PID values from the Shuffleboard entries.
     * 
     * This method reads the current values of kP, kI, kD, and kF from the
     * Shuffleboard entries and returns them as an array.
     * Call this in your periodic function and run .setPID() with it.
     * 
     * @return An array containing the current PID values in the order [kP, kI, kD, kF].
     */
    public double[] getPID() {
        return new double[] {
            kPEntry.getDouble(kP), 
            kIEntry.getDouble(kI), 
            kDEntry.getDouble(kD), 
            kFEntry.getDouble(kF)
        };
    }
}

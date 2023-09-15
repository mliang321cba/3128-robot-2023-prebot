package frc.team3128.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.utility.NAR_Shuffleboard;

/**
 * A subsystem based off of {@link PIDSubsystem} 
 * @since 2023 CHARGED UP
 * @author Mason Lam
 */
public abstract class NAR_PIDSubsystem extends SubsystemBase {
    protected final PIDController m_controller;
    protected boolean m_enabled;
    private DoubleSupplier kS, kV, kG;
    private DoubleSupplier kG_Function;
    private BooleanSupplier debug;
    private DoubleSupplier setpoint;
    private double min, max;

    /**
     * Creates a new PIDSubsystem.
     *
     * @param controller the PIDController to use
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kG The gravity gain.
     */
    public NAR_PIDSubsystem(PIDController controller, double kS, double kV, double kG) {
        m_controller = controller;
        this.kG_Function = () -> 1;
        initShuffleboard(kS, kV, kG);
        min = Double.NEGATIVE_INFINITY;
        max = Double.POSITIVE_INFINITY;
    }

    /**
     * Creates a new PIDSubsystem.
     *
     * @param controller the PIDController to use
     */
    public NAR_PIDSubsystem(PIDController controller) {
        this(controller, 0, 0, 0);
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            double output = m_controller.calculate(getMeasurement());
            output += Math.copySign(kS.getAsDouble(), output);
            output += kV.getAsDouble() * getSetpoint();
            output += kG_Function.getAsDouble() * kG.getAsDouble();
            useOutput(output, getSetpoint());
        }
    }

    private void initShuffleboard(double kS, double kV, double kG) {
        NAR_Shuffleboard.addComplex(getName(), "PID_Controller", m_controller, 0, 0);

        NAR_Shuffleboard.addData(getName(), "Enabled", ()-> isEnabled(), 1, 0);
        NAR_Shuffleboard.addData(getName(), "Measurement", ()-> getMeasurement(), 1, 1);
        NAR_Shuffleboard.addData(getName(), "Setpoint", ()-> getSetpoint(), 1, 2);

        var debugEntry = NAR_Shuffleboard.addData(getName(), "TOGGLE", false, 2, 0).withWidget("Toggle Button");
        debug = ()-> debugEntry.getEntry().getBoolean(false);
        NAR_Shuffleboard.addData(getName(), "DEBUG", ()-> debug.getAsBoolean(), 2, 1);
        setpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 2,2);

        this.kS = NAR_Shuffleboard.debug(getName(), "kS", kS, 3, 0);
        this.kV = NAR_Shuffleboard.debug(getName(), "kV", kV, 3, 1);
        this.kG = NAR_Shuffleboard.debug(getName(), "kG", kG, 3, 2);
    }

    /**
     * Returns the PIDController object controlling the subsystem
     *
     * @return The PIDController
     */
    public PIDController getController() {
        return m_controller;
    }

    /**
     * Sets constraints for the setpoint of the PID subsystem.
     * @param min The minimum setpoint for the subsystem
     * @param max The maximum setpoint for the subsystem
     */
    public void setConstraints(double min, double max) {
        this.min = min;
        this.max = max;
    }

    /**
     * Sets the function returning the value multiplied against kG
     * @param kG_Function the function multiplied to kG
     */
    public void setkG_Function(DoubleSupplier kG_Function) {
        this.kG_Function = kG_Function;
    }

    /**
     * Sets the setpoint for the subsystem.
     *
     * @param setpoint the setpoint for the subsystem
     */
    public void startPID(double setpoint) {
        enable();
        m_controller.setSetpoint(MathUtil.clamp(debug.getAsBoolean() ? this.setpoint.getAsDouble() : setpoint, min, max));
    }

    /**
     * Returns the current setpoint of the subsystem.
     *
     * @return The current setpoint
     */
    public double getSetpoint() {
        return m_controller.getSetpoint();
    }

    /**
     * Returns true if subsystem is at setpoint, false if not
     *
     * @return If subsystem is at setpoint
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    /**
     * Uses the output from the PIDController.
     *
     * @param output the output of the PIDController
     * @param setpoint the setpoint of the PIDController (for feedforward)
     */
    protected abstract void useOutput(double output, double setpoint);

    /**
     * Returns the measurement of the process variable used by the PIDController.
     *
     * @return the measurement of the process variable
     */
    protected abstract double getMeasurement();

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        m_enabled = true;
        m_controller.reset();
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        m_enabled = false;
        useOutput(0, 0);
    }

    /**
     * Returns whether the controller is enabled.
     *
     * @return Whether the controller is enabled.
     */
    public boolean isEnabled() {
        return m_enabled;
    }
}
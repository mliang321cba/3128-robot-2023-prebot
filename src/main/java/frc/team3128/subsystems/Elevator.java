package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import static frc.team3128.PositionConstants.Position;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import static frc.team3128.Constants.ElevatorConstants.*;

public class Elevator extends NAR_PIDSubsystem {

    private NAR_CANSparkMax m_elv1, m_elv2;

    private static Elevator instance;

    public static synchronized Elevator getInstance() {
        if (instance == null){
            instance = new Elevator();  
        }
        return instance;
    }

    public Elevator() {
        super(new PIDController(kP, kI, kD), kS, kV, kG);
        setConstraints(MIN_DIST, MAX_DIST);
        configMotors();
        initShuffleboard(kS, kV, kG);
        m_controller.setTolerance(ELV_TOLERANCE);
        resetEncoder();
    }

    public void startPID(Position position) {
        startPID(position.elvDist);
    }

    private void configMotors() {
        m_elv1 = new NAR_CANSparkMax(ELV1_ID);
        m_elv2 = new NAR_CANSparkMax(ELV2_ID);

        m_elv1.setInverted(false);
        m_elv2.setInverted(true);

        m_elv1.setIdleMode(IdleMode.kCoast);
        m_elv2.setIdleMode(IdleMode.kCoast);

        m_elv1.setSmartCurrentLimit(CURRENT_LIMIT); 
        m_elv2.setSmartCurrentLimit(CURRENT_LIMIT);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        final double power = MathUtil.clamp(output / 12.0, -1, 1);
        m_elv1.set(power);
        m_elv2.set(power);
    }

    public void set(double power) {
        disable();
        m_elv1.set(power);
        m_elv2.set(power);
    }

    public void stop() {
        set(0);
    }

    public void resetEncoder() {
        m_elv2.setSelectedSensorPosition(0);
    }

    @Override
    public double getMeasurement() {
        return m_elv2.getSelectedSensorPosition() / GEAR_RATIO * SPOOL_CIRCUMFERENCE;
    }

}

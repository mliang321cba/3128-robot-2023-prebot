package frc.team3128.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import static frc.team3128.Constants.WristConstants.*;

import static frc.team3128.PositionConstants.Position;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax.EncoderType;

public class Wrist extends NAR_PIDSubsystem {
    public NAR_CANSparkMax m_wrist;

    private static Wrist instance;

    public static synchronized Wrist getInstance() {
        if (instance == null){
            instance = new Wrist();  
        }
        return instance;
    }

    public Wrist() {
        super(new PIDController(kP, kI, kD), kS, kV, kG);
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
        setConstraints(MIN_ANGLE, MAX_ANGLE);
        configMotor();
        initShuffleboard(kS, kV, kG);
        m_controller.setTolerance(WRIST_TOLERANCE);
    }

    public void startPID(Position position) {
        startPID(position.wristAngle);
    }

    private void configMotor() {
        m_wrist = new NAR_CANSparkMax(WRIST_ID, EncoderType.Relative, MotorType.kBrushless);
        m_wrist.setInverted(false);
        m_wrist.setIdleMode(IdleMode.kCoast);
        m_wrist.setSmartCurrentLimit(40);
        resetEncoder();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_wrist.set(MathUtil.clamp(output / 12.0, -1, 1));
    }

    @Override
    public double getMeasurement() {
        return m_wrist.getSelectedSensorPosition() * ROTATION_TO_DEGREES / GEAR_RATIO;
    }

    public void resetEncoder() {
        m_wrist.setSelectedSensorPosition(90 * GEAR_RATIO / ROTATION_TO_DEGREES);
    }

    public void set(double power) {
        disable();
        m_wrist.set(power);
    }

}

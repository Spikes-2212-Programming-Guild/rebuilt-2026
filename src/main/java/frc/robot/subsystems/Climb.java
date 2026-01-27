package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Climb extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "climb";

    private static final boolean LEFT_MOTOR_INVERTED = false;

    private static final double MAX_ARM_POSITION = -1;
    private static final double MIN_ARM_POSITION = -1;

    //@TODO check if measuring the distance per rotation instead of calculating works better
    private static final double GEAR_RATIO = -1;
    private static final double SPOOL_DIAMETER_METERS = -1;
    private static final double SPOOL_CIRCUMFERENCE_METERS = Math.PI * SPOOL_DIAMETER_METERS;
    private static final double DISTANCE_PER_ROTATION_METERS = SPOOL_CIRCUMFERENCE_METERS * GEAR_RATIO;

    private final TalonFXWrapper leftTalonFX;
    private final TalonFXWrapper rightTalonFX;

    private static Climb instance;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.CLIMB_TALON_FX_LEFT_ID),
                    new TalonFXWrapper(RobotMap.CAN.CLIMB_TALON_FX_RIGHT_ID),
                    LEFT_MOTOR_INVERTED
            );
        }
        return instance;
    }

    private Climb(String namespaceName, TalonFXWrapper leftTalonFX,
                  TalonFXWrapper rightTalonFx, boolean leftMotorInverted) {
        super(namespaceName, leftTalonFX);
        this.leftTalonFX = leftTalonFX;
        this.rightTalonFX = rightTalonFx;
        leftTalonFX.setEncoderConversionFactor(DISTANCE_PER_ROTATION_METERS);
        rightTalonFx.follow(leftTalonFX, leftMotorInverted);
        configureDashboard();
    }

    @Override
    public boolean canMove(double speed) {
        return (
            (speed > 0 && leftTalonFX.getPosition() != MAX_ARM_POSITION) ||
            (speed < 0 && leftTalonFX.getPosition() != MIN_ARM_POSITION)
        );
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("left position", leftTalonFX::getPosition);
        namespace.putNumber("right position", rightTalonFX::getPosition);
    }
}

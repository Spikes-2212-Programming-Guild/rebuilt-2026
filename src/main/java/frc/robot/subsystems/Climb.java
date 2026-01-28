package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Climb extends SmartMotorControllerGenericSubsystem {

    public enum ArmPose {

        TOP(-1), BOTTOM(-1);

        private final double positionMeters;

        public double getPositionMeters() {
            return positionMeters;
        }

        ArmPose(double position) {
            this.positionMeters = position;
        }
    }

    private static final String NAMESPACE_NAME = "climb";

    private static final boolean LEFT_MOTOR_INVERTED = false;

    private static final double MAX_ARM_POSITION_METERS = -1;
    private static final double MIN_ARM_POSITION_METERS = -1;

    private static final double GEAR_RATIO = -1;
    private static final double SPOOL_DIAMETER_METERS = -1;

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
        rightTalonFx.follow(leftTalonFX, leftMotorInverted);
        configureRelativeEncoder();
        configureDashboard();
    }

    private void configureRelativeEncoder() {
        double spoolCircumference = SPOOL_DIAMETER_METERS * Math.PI;
        leftTalonFX.setEncoderConversionFactor(spoolCircumference * GEAR_RATIO);
    }

    @Override
    public boolean canMove(double speed) {
        return (
            (speed > 0 && leftTalonFX.getPosition() != MAX_ARM_POSITION_METERS) ||
            (speed < 0 && leftTalonFX.getPosition() != MIN_ARM_POSITION_METERS)
        );
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("left position", leftTalonFX::getPosition);
        namespace.putNumber("right position", rightTalonFX::getPosition);

        namespace.putNumber("left velocity", leftTalonFX::getVelocity);
        namespace.putNumber("right velocity", rightTalonFX::getVelocity);
    }
}

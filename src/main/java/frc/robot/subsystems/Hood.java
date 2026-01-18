package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Hood extends SmartMotorControllerGenericSubsystem {

    public enum HoodPose {

        TOSS_POSE(-1), MAX_ANGLE(-1), MIN_ANGLE(-1);

        public final double neededPitch;

        HoodPose(double neededPitch) {
            this.neededPitch = neededPitch;
        }
    }

    public static final double HOOD_SPEED = -1.0;

    private static final String NAMESPACE_NAME = "hood";
    private static final double GEAR_RATIO = -1.0;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DISTANCE_PER_PULSE = GEAR_RATIO * DEGREES_IN_ROTATIONS;

    private final TalonFXWrapper talonFX; // I used talonFX as a place holder that will be change
    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    private static Hood instance;

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.HOOD_MOTOR),
                    new DigitalInput(RobotMap.DIO.HOOD_TOP_LIMIT),
                    new DigitalInput(RobotMap.DIO.HOOD_BOTTOM_LIMIT));
        }
        return instance;
    }

    private Hood(String namespaceName, TalonFXWrapper talonFX, DigitalInput topLimit, DigitalInput bottomLimit) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
        talonFX.setEncoderConversionFactor(DISTANCE_PER_PULSE);
        configureDashboard();
    }

    @Override
    public boolean canMove(double speed) {
        return !((bottomLimit.get() && speed < 0) || (topLimit.get() && speed > 0));
    }

    public void calibrateEncoderPosition() {
        if (topLimit.get()) {
            talonFX.setPosition(HoodPose.MAX_ANGLE.neededPitch);
        } else if (bottomLimit.get()) {
            talonFX.setPosition(HoodPose.MIN_ANGLE.neededPitch);
        }
    }

    public void configureDashboard() {
        namespace.putBoolean("top limit", topLimit::get);
        namespace.putBoolean("bottom limit", bottomLimit::get);
        namespace.putNumber("hood pose", talonFX::getPosition);
    }
}

package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Shooter extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "shooter";

    private final TalonFXWrapper rightMotor;
    private final TalonFXWrapper middleMotor;
    private final TalonFXWrapper leftMotor;

    private static Shooter instance;

    public static Shooter getInstance() {
        if(instance == null){
            instance = new Shooter(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_UPPER_TALON_FX_ID),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_MIDDLE_TALON_FX_ID),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_LOWER_TALON_FX_ID));
        }
        return instance;
    }

    private Shooter(String namespaceName, TalonFXWrapper rightMotor, TalonFXWrapper middleMotor,
                    TalonFXWrapper leftMotor) {
        super(namespaceName, rightMotor, middleMotor, leftMotor);
        this.rightMotor = rightMotor;
        this.middleMotor = middleMotor;
        this.leftMotor = leftMotor;
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("upper kraken speed", rightMotor::getVelocity);
        namespace.putNumber("middle kraken speed", middleMotor::getVelocity);
        namespace.putNumber("lower kraken speed", leftMotor::getVelocity);
    }
}

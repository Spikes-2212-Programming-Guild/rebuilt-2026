package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class ShootToHub extends SequentialCommandGroup {

    private static final Supplier<Double> LINEAR_EQUATION_M_FACTOR = () -> 0.636;
//    private static final Supplier<Double> LINEAR_EQUATION_M_FACTOR = Robot.namespace.addConstantDouble("m factor", 0.636);
//
    private static final Supplier<Double> LINEAR_EQUATION_B_FACTOR = () -> 1.7;
//    private static final Supplier<Double> LINEAR_EQUATION_B_FACTOR = Robot.namespace.addConstantDouble("b factor", 1.7);

    private static final Supplier<Double> DISTANCE_FROM_CAMERA_TO_SHOOTER = () -> 0.07;
//    private static final Supplier<Double> DISTANCE_FROM_CAMERA_TO_SHOOTER = Robot.namespace.addConstantDouble("offset", 0.15);

    private static final double FIRST_WAIT_TIME = 0.5;
    private static final double SECOND_WAIT_TIME = 10; // doesn't matter

    public ShootToHub(Supplier<Double> speed) {
        addCommands(
                new ShootWithPID(Shooter.getInstance(), this::calculateSpeed, FIRST_WAIT_TIME).withTimeout(4),
                new ParallelCommandGroup(
                        new SpinMagazine(),
                        new Transport(),
                        new ShootWithPID(Shooter.getInstance(), this::calculateSpeed, SECOND_WAIT_TIME) {

                            @Override
                            public boolean isFinished() {
                                return false;
                            }
                        }
                )
        );
    }

    private double calculateSpeed() {
        double distance = VisionService.getInstance().getZ();
        double mFactor = LINEAR_EQUATION_M_FACTOR.get();
        double bFactor = LINEAR_EQUATION_B_FACTOR.get();
        double offset = DISTANCE_FROM_CAMERA_TO_SHOOTER.get();
        return mFactor * (distance + offset) + bFactor;
    }
}

package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.storage.SpinMagazine;
import frc.robot.commands.storage.Transport;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class ShootToHub extends SequentialCommandGroup {

    private static final Supplier<Double> LINEAR_EQUATION_M_FACTOR = () -> 0.636;
//    private static final Supplier<Double> LINEAR_EQUATION_M_FACTOR = Robot.namespace.addConstantDouble("m factor", 0.636);

    private static final Supplier<Double> LINEAR_EQUATION_B_FACTOR = () -> 1.7;
//    private static final Supplier<Double> LINEAR_EQUATION_B_FACTOR = Robot.namespace.addConstantDouble("b factor", 1.7);

    private static final Supplier<Double> DISTANCE_FROM_CAMERA_TO_SHOOTER = () -> 0.15;
//    private static final Supplier<Double> DISTANCE_FROM_CAMERA_TO_SHOOTER = Robot.namespace.addConstantDouble("offset", 0.15);

    private static final double FIRST_WAIT_TIME = 0.75;
    private static final double SECOND_WAIT_TIME = 10; // doesn't matter

    public ShootToHub() {
        addCommands(
                new ShootWithPID(() -> (LINEAR_EQUATION_M_FACTOR.get() *
                        (VisionService.getInstance().getZ() + DISTANCE_FROM_CAMERA_TO_SHOOTER.get())
                        + LINEAR_EQUATION_B_FACTOR.get()), FIRST_WAIT_TIME) {

                }.withTimeout(3),
                new ParallelCommandGroup(
                        new SpinMagazine(),
                        new Transport(),
                        new ShootWithPID(() -> (LINEAR_EQUATION_M_FACTOR.get() *
                                (VisionService.getInstance().getZ() + DISTANCE_FROM_CAMERA_TO_SHOOTER.get()) +
                                LINEAR_EQUATION_B_FACTOR.get()), SECOND_WAIT_TIME) {

                            @Override
                            public boolean isFinished() {
                                return false;
                            }
                        }
                )
        );
    }
}

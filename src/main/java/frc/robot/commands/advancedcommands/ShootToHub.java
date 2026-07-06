package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;
import frc.robot.utils.VisionService;

public class ShootToHub extends SequentialCommandGroup {

    private static final double LINEAR_EQUATION_M_FACTOR = 0.636;
    private static final double LINEAR_EQUATION_B_FACTOR = 1.7;
    private static final double DISTANCE_FROM_CAMERA_TO_SHOOTER = 0.07;

    private static final double FIRST_WAIT_TIME = 0.5;
    private static final double SECOND_WAIT_TIME = 20;

    public ShootToHub() {
        addCommands(
                new ShootWithPID(this::calculateSpeed, FIRST_WAIT_TIME).withTimeout(4),
                new ParallelCommandGroup(
                        new SpinMagazine(),
                        new Transport(),
                        new ShootWithPID(this::calculateSpeed, SECOND_WAIT_TIME)
                )
        );
    }

    private double calculateSpeed() {
        double distance = VisionService.getInstance().getZ();
        double mFactor = LINEAR_EQUATION_M_FACTOR;
        double offset = DISTANCE_FROM_CAMERA_TO_SHOOTER;
        return mFactor * (distance + offset) + LINEAR_EQUATION_B_FACTOR;
    }
}

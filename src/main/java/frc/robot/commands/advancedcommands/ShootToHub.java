package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class ShootToHub extends SequentialCommandGroup {

    private static final double LINEAR_EQUATION_M_FACTOR = 0.636;
    private static final double LINEAR_EQUATION_B_FACTOR = 1.7;

    public ShootToHub() {
        addCommands(
                new ShootWithPID(this::calculateSpeed, 0.75).withTimeout(4),
                new ParallelCommandGroup(
                        new SpinMagazine(),
                        new Transport(),
                        new ShootWithPID(this::calculateSpeed, 20) {
                            @Override
                            public void end(boolean interrupted) {
                                Shooter.getInstance().stop();
                            }

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
        double mFactor = LINEAR_EQUATION_M_FACTOR;
        Supplier<Double> offset = Robot.offset;
        return mFactor * (distance + offset.get()) + LINEAR_EQUATION_B_FACTOR;
    }
}

package frc.robot.util.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record StandardDeviations(double translationXStdDev, double translationYStdDev, double rotationStdDev) {

    public Matrix<N3, N1> toMatrix() {
        return VecBuilder.fill(
                translationXStdDev,
                translationYStdDev,
                rotationStdDev
        );
    }
}

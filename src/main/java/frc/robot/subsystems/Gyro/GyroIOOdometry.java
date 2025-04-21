package frc.robot.subsystems.Gyro;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class GyroIOOdometry implements GyroIO {
    private SwerveDriveKinematics kinematics;
    private Supplier<SwerveModulePosition[]> modulePositionsSupplier = null;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[]{
        new SwerveModulePosition(0, Rotation2d.kZero),
        new SwerveModulePosition(0, Rotation2d.kZero),
        new SwerveModulePosition(0, Rotation2d.kZero),
        new SwerveModulePosition(0, Rotation2d.kZero)
    };
    private double accumulatedYawRad = 0.0;

    public GyroIOOdometry(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    public void setModulePositionsSupplier(Supplier<SwerveModulePosition[]> modulePositionsSupplier) {
        this.modulePositionsSupplier = modulePositionsSupplier;
    }

    public double getYaw() {
        return Math.toDegrees(accumulatedYawRad);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromRadians(accumulatedYawRad);
    }

    public void reset() {
        accumulatedYawRad = 0.0;
    }

    public double getPitch() {return 0.0;};
    public double getRoll() {return 0.0;};
    public double getPitchVelocity() {return 0.0;};
    public double getYawVelocity() {return 0.0;};
    public double getRollVelocity() {return 0.0;};
    public double getAccelerationX() {return 0.0;};
    public double getAccelerationY() {return 0.0;};
    public double getAccelerationZ() {return 0.0;};
    public double getVelocityX() {return 0.0;};
    public double getVelocityY() {return 0.0;};
    public double getVelocityZ() {return 0.0;};

    @Override
    public void periodic() {
        if (modulePositionsSupplier == null) return;
        SwerveModulePosition[] currentModulePositions = modulePositionsSupplier.get();
        Twist2d twist = kinematics.toTwist2d(lastModulePositions, currentModulePositions);
        accumulatedYawRad =+ twist.dtheta;
        lastModulePositions = currentModulePositions;
    }
}

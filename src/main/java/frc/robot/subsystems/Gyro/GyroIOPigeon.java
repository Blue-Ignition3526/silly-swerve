package frc.robot.subsystems.Gyro;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import lib.team3526.constants.CTRECANDevice;

public class GyroIOPigeon implements GyroIO {
    public Pigeon2 gyro;

    public GyroIOPigeon(CTRECANDevice device) {
        gyro = new Pigeon2(device.getDeviceID(), device.getCanbus());
    }

    public double getPitch() {
        return gyro.getPitch().refresh().getValue().in(Degrees);
    }

    public double getYaw() {
        return gyro.getYaw().refresh().getValue().in(Degrees);
    }

    public double getRoll() {
        return gyro.getRoll().refresh().getValue().in(Degrees);
    }

    public double getPitchVelocity() {
        return gyro.getAngularVelocityXWorld().refresh().getValue().in(DegreesPerSecond);
    }

    public double getYawVelocity() {
        return 0;
    }

    public double getRollVelocity() {
        return gyro.getAngularVelocityZWorld().refresh().getValue().in(DegreesPerSecond);
    }

    public double getAccelerationX() {
        return 0;
    }

    public double getAccelerationY() {
        return 0;
    }

    public double getAccelerationZ() {
        return 0;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }
    
    public void reset() {
        gyro.reset();
    }

    public double getVelocityX() {
        // TODO Auto-generated method stub
        return 0;
    }

    public double getVelocityY() {
        // TODO Auto-generated method stub
        return 0;
    }

    public double getVelocityZ() {
        // TODO Auto-generated method stub
        return 0;
    }
}

package frc.robot.utils;

public class Trapezoid {

    public double MaxError = 0.01;
    public double MaxAcceleration = 1;
    public double MaxVelocity = 0.7;
    public double KP = 2;
    public double KS = 0.06;
    public double KG = 0.23;
    public double KV = 16;
    public double KA = 2;

    public double AccelerationTime = MaxVelocity / MaxAcceleration;
    public double AccelerationDistance = 0.5 * MaxAcceleration * AccelerationTime * AccelerationTime;
    public double MaxVelocityChange = MaxVelocity * 0.04;

    public Trapezoid(double maxError, double maxAcceleration, double maxVelocity, double kP, double kS, double kG,
            double kV, double kA) {
        MaxError = maxError;
        MaxAcceleration = maxAcceleration;
        MaxVelocity = maxVelocity;
        KP = kP;
        KS = kS;
        KG = kG;
        KV = kV;
        KA = kA;
        AccelerationTime = MaxVelocity / MaxAcceleration;
        AccelerationDistance = 0.5 * MaxAcceleration * AccelerationTime * AccelerationTime;
        MaxVelocityChange = MaxVelocity * 0.04;
    }

    public double calculateVolts(double currentPosition, double currentVelcity, double targetPosition) {
        double targetVelcity = calculateVelocity(currentPosition, currentVelcity, targetPosition);
        double acceleration = calculateAcceleration(currentVelcity, targetVelcity);
        double volts = calculateVoltage(currentVelcity, targetVelcity, acceleration);
        return volts;
    }

    public double calculateVoltage(double currentVelcity, double targetVelcity, double acceleration) {
        return KS * Math.signum(targetVelcity) + KG + targetVelcity * KV + acceleration * KA + (targetVelcity - currentVelcity) * KP;
    }
    
    public double calculateAcceleration(double currentVelcity, double targetVelocity) {
        return 
            targetVelocity == 0 ? 0:
            targetVelocity  > currentVelcity + MaxVelocityChange ? MaxAcceleration:
            targetVelocity  < currentVelcity - MaxVelocityChange ? -MaxAcceleration:
            0;
    }

    public double calculateVelocity(double currentPosition, double currentVelcity, double targetPosition) {
        double error = targetPosition - currentPosition; // position error
        if(Math.abs(error) < MaxError) { // at position
            return 0;
        } else if(Math.abs(error) < AccelerationDistance) { // deacceleration
            return Math.signum(error) * Math.sqrt(2 * Math.abs(error) * MaxAcceleration);
        } else if(error < 0) { // going down
            return Math.max(-MaxVelocity, currentVelcity - MaxVelocityChange);
        } else { // going up
            return  Math.min(MaxVelocity, currentVelcity + MaxVelocityChange);
        }
    }

}

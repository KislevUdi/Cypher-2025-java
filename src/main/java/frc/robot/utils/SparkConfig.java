package frc.robot.utils;


/** 
 * Class to hold all Talon FX/SRX configuration
 * Applicable to Phoenix 6
 *  */
public class SparkConfig {
    public int id;                  // Canbus ID
    public String name;             // Name of the motor - used to logging

    public double maxVolt = 12;     // Max Volt allowed
    public double minVolt = -12;    // Min Vols allowed
    public double maxCurrent = 40;  // Max current allowed
    public double rampUpTime = 0.3;   // max power change time from 0 to full. 

    public boolean brake = true;    // brake/coast
    public double motorRatio = 1;   // motor to mechanism ratio
    public boolean inverted = false; // if to invert motor

    public closeLoopParam pid = new closeLoopParam(0, 0, 0,0); // close loop argument - PID + FF
    public closeLoopParam pid1 = null; // pid for slot 1
    public closeLoopParam pid2 = null; // pid for slot 2

    public double maxVelocity = 0;
    public double minVelocity = 0;
    public double maxAcceleration = 0;


    /** 
    * Class to hold closed loop param
    *  */
    class closeLoopParam { // calculate volts - not -1 to 1 !!!
        double kp;  
        double ki;
        double kd;
        double kf;

        closeLoopParam(double kp, double ki, double kd, double kf) {
            this.kd = kd;
            this.ki = ki;
            this.kp = kp;
            this.kf = kf;
        }
    }

    /** 
     * Constructor
     * @param id - canbus ID
     * @param canbus - Name of canbus
     * @param name - name of motor for logging
     */
    public SparkConfig(int id, String name) {
        this.id = id;
        this.name = name;
    }

    public SparkConfig(int id, String name,SparkConfig config) {
        this.id = id;
        this.name = name;
        this.maxVolt = config.maxVolt;
        this.minVolt = config.minVolt;
        this.maxCurrent = config.maxCurrent;
        this.rampUpTime = config.rampUpTime;
        this.brake = config.brake;
        this.motorRatio = config.motorRatio;
        this.inverted = config.inverted;
        this.pid = config.pid;
        this.pid1 = config.pid1;
        this.pid2 = config.pid2;
        this.maxVelocity = config.maxVelocity;
        this.minVelocity = config.minVelocity;
        this.maxAcceleration = config.maxAcceleration;
    }

    
    /** 
     * @param maxVolt
     * @param minVolt
     * @return SparkConfig
     */
    public SparkConfig withVolts(double maxVolt, double minVolt) {
        this.maxVolt = maxVolt;
        this.minVolt = minVolt;
        return this;
    }
    
    /** 
     * @param maxCurrent
     * @param treshold
     * @param trigerTime
     * @return SparkConfig
     */
    public SparkConfig withCurrent(double maxCurrent) {
        this.maxCurrent = maxCurrent;
        return this;
    }

    
    /** 
     * @param brake
     * @return SparkConfig
     */
    public SparkConfig withBrake(boolean brake) {
        this.brake = brake;
        return this;
    }

    /** 
     * @param invert
     * @return SparkConfig
     */
    public SparkConfig withInvert(boolean invert) {
        this.inverted = invert;
        return this;
    }

    /** 
     * @param rampTime
     * @return SparkConfig
     */
    public SparkConfig withRampTime(double rampTime) {
        this.rampUpTime = rampTime;
        return this;
    }

    /** 
     * @param ratio - motor to mechanism ratio
     * @return SparkConfig
     */
    public SparkConfig withMotorRatio(double ratio) {
        this.motorRatio *= ratio;
        return this;
    }

    public SparkConfig withMeterMotor(double circonference) {
        this.motorRatio *= 1 / circonference;
        return this;
    }

    public SparkConfig withRadiansMotor() {
        this.motorRatio *= 1 / (Math.PI * 2);
        return this;
    }

    /** 
     * @param kp
     * @param ki
     * @param kd
     * @return SparkConfig
     */
    public SparkConfig withPID(double kp, double ki, double kd, double kf) {
        pid = new closeLoopParam(kp, ki, kd, kf);
        return this;
    }



    /** 
     * @param kp
     * @param ki
     * @param kd
     * @param ks
     * @param kv
     * @return SparkConfig
     */
    public SparkConfig withPID1(double kp, double ki, double kd, double kf) {
        pid1 = new closeLoopParam(kp, ki, kd, kf);
        return this;
    }

    /** 
     * @param kp
     * @param ki
     * @param kd
     * @param ks
     * @param kv
     * @return SparkConfig
     */
    public SparkConfig withPID2(double kp, double ki, double kd, double kf) {
        pid2 = new closeLoopParam(kp, ki, kd, kf);
        return this;
    }    

    public SparkConfig withVelocity(double maxVelocity, double minVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.minVelocity = minVelocity;
        return this;
    }
    

}
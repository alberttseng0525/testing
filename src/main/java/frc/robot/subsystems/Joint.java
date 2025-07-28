package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.databind.node.DoubleNode;
import com.ctre.phoenix6.configs.TalonFXConfiguration ;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import java.io.ObjectInputFilter.Config;
import java.lang.module.Configuration;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Joint extends SubsystemBase {
    private TalonFX joint = new TalonFX(Constants.joint.MOTOR_ID);
    private CANcoder jointCaNcoder = new CANcoder(Constants.joint.CANCODER_ID);
    private static Joint system;

    public Joint() {
        joint.getConfigurator().apply(jointConfiguration());
        joint.setNeutralMode(NeutralModeValue.Brake); // Set the motor to brake mode
        jointCaNcoder.getConfigurator().apply(jointCaNcoderConfiguration());
    }

    public Command moveTo(double desireposition) {
        double setpoint = desireposition/360;
        return this.run(() -> {
            joint.setPosition(setpoint); 
        });
    }

    public void moveJoint(double setpoint){
        joint.setControl(new PositionDutyCycle(setpoint));
        SmartDashboard.putNumber("Joint setpoint",setpoint); // Display the joint position in degrees

    }

    public double getJointPosition() {
        return joint.getPosition().getValueAsDouble();      // Get the joint position in degrees
    }

    public void stopjoint(){
        joint.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("joint position", getJointPosition());
    }


    private TalonFXConfiguration jointConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();            // Add necessary configuration settings here
        
        config.Feedback.SensorToMechanismRatio = 1.0; // Set the sensor to mechanism ratio
        config.Feedback.RotorToSensorRatio =  Constants.joint.CANCORDER_GEAR_RATIO; // Set the motor to sensor ratio

        config.Slot0 = new Slot0Configs()
            .withKS(Constants.joint.KS) // Set the feedforward gain
            .withKV(Constants.joint.KV) // Set the velocity gain       
            .withKA(Constants.joint.KA) // Set the acceleration gain
            .withKG(Constants.joint.KG) // Set the gravity compensation gain
            .withKP(Constants.joint.KP) // Set the proportional gain
            .withKI(Constants.joint.KI) // Set the integral gain
            .withKD(Constants.joint.KD); // Set the derivative gain
            //.withKf(Constants.joint.KF)// Set the feedforward gain

        config.Slot0.GravityType = Constants.joint.GRAVITY_TYPE; // Set the gravity compensation type

        config.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.joint.CRUISE_VELOCITY) // Set the motion magic cruise velocity
            .withMotionMagicAcceleration(Constants.joint.ACCELERATION) // Set the motion magic acceleration
            .withMotionMagicJerk(Constants.joint.JERK); // Set the motion magic jerk

        config.CurrentLimits.StatorCurrentLimit = Constants.joint.STATOR_CURRENT_LIMIT; // Set the stator current limit in Amperes
        config.CurrentLimits.SupplyCurrentLimit = Constants.joint.SUPPLY_CURRENT_LIMIT; // Set the supply current limit in Amperes
        config.CurrentLimits.SupplyCurrentLowerTime = Constants.joint.SUPPLY_CURRENT_LOWER_TIME; // Set the supply current lower time in seconds
       
        config.Feedback.FeedbackRemoteSensorID =  Constants.joint.CANCODER_ID; // Set the CANCoder ID for feedback
        config.Feedback.FeedbackSensorSource =  FeedbackSensorSourceValue.RemoteCANcoder; // Set the feedback sensor source to CANCoder

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Set the motor output to counter-clockwise positive
       
        return config;
    }
     
    private CANcoderConfiguration jointCaNcoderConfiguration() {
        CANcoderConfiguration config = new CANcoderConfiguration();          // Add necessary configuration settings here

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = Constants.joint.ABSOLUTE_SENSOR_DISCONTINUITY_POINT; // Set the absolute sensor discontinuity point in degrees
        config.MagnetSensor.MagnetOffset = Constants.joint.MAGNET_OFFSET; // Set the absolute sensor offset in degrees
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // Set the sensor direction to clockwise positive

        return config;
    }

    public static Joint system(){
        if (system == null) {
            system = new Joint();
        }

        return system;
    }
}

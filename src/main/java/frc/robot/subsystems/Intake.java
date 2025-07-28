package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Intake extends SubsystemBase {
    private TalonFX motor = new TalonFX(Constants.Intake.MOTOR_ID);

    public enum IntakeState {
        intake,carry,L123,L4,Default
    }
    
    private Map<IntakeState, Double> stateMap = new HashMap<>();
    private IntakeState state = IntakeState.Default;

    public  void intake() {

        // Initialize the state map with default values
        stateMap.put(IntakeState.intake, Constants.intake.intake);
        stateMap.put(IntakeState.carry, Constants.intake.carry);
        stateMap.put(IntakeState.L123, Constants.intake.L123);
        stateMap.put(IntakeState.L4, Constants.intake.L4);
        stateMap.put(IntakeState.Default, Constants.intake.Default);
        // Set the motor to brake mode
        motor.setNeutralMode(NeutralModeValue.Brake); 
        this.setDefaultCommand(this.set(IntakeState.Default).repeatedly());  

    }

    public Command set(IntakeState state){
        return this.run(() -> motor.setVoltage(stateMap.get(state)*12));
    }

    public Command estop() {
        state = IntakeState.Default;
        return this.run(() -> {
            motor.setVoltage(0);
        });
    }


}

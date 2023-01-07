package com.team7419;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public abstract class Initers {

    public static void initVictors(VictorSPX...victors){
        for (VictorSPX victor : victors) {
			victor.neutralOutput();
		    victor.setSensorPhase(false);
		    victor.configNominalOutputForward(0, 0);
		    victor.configNominalOutputReverse(0, 0);
		    victor.configClosedloopRamp(.2, 0);
		}
	}
	
	public static void initTalons(TalonSRX...talons){
		for (TalonSRX talon : talons){
			talon.neutralOutput();
			talon.setSensorPhase(false);
			talon.configNominalOutputForward(0, 0);
			talon.configNominalOutputReverse(0, 0);
			talon.configClosedloopRamp(.2, 0);
		}
	}
}
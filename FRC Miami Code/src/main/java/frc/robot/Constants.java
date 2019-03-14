package frc.robot;
public class Constants {
	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 30;
	public static boolean kSensorPhase = true;
	public static boolean kMotorInvert = true;
	//En alttaki koddan TalonSRX'in çalıştıracağı PID değerlerini belirliyoruz, bu kısım değiştirilebilir-düzenlenebilir.
		//Gains(kp, ki, kd, kf, izone, peak output);
	static final Gains kGains = new Gains(6, 0, 0, 0, 0, 1);
}
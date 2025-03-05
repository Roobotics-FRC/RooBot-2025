package frc.robot;

public class Constants {
    public static final class Positions{
        public static final double L0 = 0.5;

        public static final double L2 = 24;
        public static final double L3 = 38;
        public static final double L4 = 67;
        public static final double L4E = 69;

        public static final double L2A = 16;
        public static final double L2AE = 24;
        public static final double L3A = 30;
        public static final double L3AE = 38;
    }

    public static final class WaitTimes{
        public static final double scoreWait = 0.5;
    }
    public static final class PID{
        public static final double translationalKP = 0.53;
        public static final double translationalKI = 0.001;
        public static final double translationalKD = 0.02;
        public static final double thanslationalIZone = 0.5;

        public static final double rotationalKP = 7;
        public static final double rotationalKI = 0;
        public static final double rotationalKD = 0;
        public static final double rotationalIZone = 0;

        public static final double rotationalTolerance = 0;
        public static final double translationalTolerance = 0;
    }
}

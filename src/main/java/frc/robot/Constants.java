package frc.robot;

public class Constants {
    public static final class Positions{
        public static final double L0 = 0.5;

        public static final double L2 = 20;
        public static final double L3 = 39;
        public static final double L4 = 67;
        public static final double L4E = 69;

        public static final double L2A = 16;
        public static final double L2AE = 24;
        public static final double L3A = 32;
        public static final double L3AE = 40;

        public static final double LeftFeeder = -55;
        public static final double RightFeeder = 55;
    }

    public static final class WaitTimes{
        public static final double scoreWait = 0.5;
    }
    public static final class PID{
        public static final double translationalKP = 0.53;
        public static final double translationalKI = 0.001;
        public static final double translationalKD = 0.02;
        public static final double thanslationalIZone = 0.5;

        public static final double rotationalKP = 0.02;
        public static final double rotationalKI = 0;
        public static final double rotationalKD = 0;
        public static final double rotationalIZone = 0;

        public static final double rotationalTolerance = 0.05;
        public static final double translationalTolerance = 0.1;
    }

    public static final class Offsets{
        public static final double xRief = -0.35;
        public static final double yRief = 0.13;
    }
}

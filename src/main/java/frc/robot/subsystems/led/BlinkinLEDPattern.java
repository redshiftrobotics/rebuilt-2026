package frc.robot.subsystems.led;

/**
 * @brief LED patterns and solid colors for REV Blinkin
 * @link https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
 */
public enum BlinkinLEDPattern {
    // Off
    OFF(1995), // same as SolidColors.BLACK

    // Fixed Palette - Rainbow
    RAINBOW_PALETTE(1005),
    PARTY_PALETTE(1015),
    OCEAN_PALETTE(1025),
    LAVA_PALETTE(1035),
    FOREST_PALETTE(1045),

    // Fixed Palette - Rainbow with Glitter
    RAINBOW_WITH_GLITTER(1055),

    // Fixed Palette - Confetti
    CONFETTI(1065),

    // Fixed Palette - Shot
    SHOT_RED(1075),
    SHOT_BLUE(1085),
    SHOT_WHITE(1095),

    // Fixed Palette - Sinelon
    SINELON_RAINBOW(1105),
    SINELON_PARTY(1115),
    SINELON_OCEAN(1125),
    SINELON_LAVA(1135),
    SINELON_FOREST(1145),

    // Fixed Palette - Beats per Minute
    BPM_RAINBOW(1155),
    BPM_PARTY(1165),
    BPM_OCEAN(1175),
    BPM_LAVA(1185),
    BPM_FOREST(1195),

    // Fixed Palette - Fire
    FIRE_MEDIUM(1205),
    FIRE_LARGE(1215),

    // Fixed Palette - Twinkles
    TWINKLES_RAINBOW(1225),
    TWINKLES_PARTY(1235),
    TWINKLES_OCEAN(1245),
    TWINKLES_LAVA(1255),
    TWINKLES_FOREST(1265),

    // Fixed Palette - Color Waves
    COLORWAVES_RAINBOW(1275),
    COLORWAVES_PARTY(1285),
    COLORWAVES_OCEAN(1295),
    COLORWAVES_LAVA(1305),
    COLORWAVES_FOREST(1315),

    // Fixed Palette - Larson Scanner
    LARSON_RED(1325),
    LARSON_GRAY(1335),

    // Fixed Palette - Light Chase
    CHASE_RED(1345),
    CHASE_BLUE(1355),
    CHASE_GRAY(1365),

    // Fixed Palette - Heartbeat
    HEARTBEAT_RED(1375),
    HEARTBEAT_BLUE(1385),
    HEARTBEAT_WHITE(1395),
    HEARTBEAT_GRAY(1405),

    // Fixed Palette - Breath
    BREATH_RED(1415),
    BREATH_BLUE(1425),
    BREATH_GRAY(1435),

    // Fixed Palette - Strobe
    STROBE_RED(1445),
    STROBE_BLUE(1455),
    STROBE_GOLD(1465),
    STROBE_WHITE(1475),

    // Color 1 Pattern
    C1_BLEND_TO_BLACK(1485),
    C1_LARSON(1495),
    C1_CHASE(1505),
    C1_HEARTBEAT_SLOW(1515),
    C1_HEARTBEAT_MEDIUM(1525),
    C1_HEARTBEAT_FAST(1535),
    C1_BREATH_SLOW(1545),
    C1_BREATH_FAST(1555),
    C1_SHOT(1565),
    C1_STROBE(1575),

    // Color 2 Pattern
    C2_BLEND_TO_BLACK(1585),
    C2_LARSON(1595),
    C2_CHASE(1605),
    C2_HEARTBEAT_SLOW(1615),
    C2_HEARTBEAT_MEDIUM(1625),
    C2_HEARTBEAT_FAST(1635),
    C2_BREATH_SLOW(1645),
    C2_BREATH_FAST(1655),
    C2_SHOT(1665),
    C2_STROBE(1675),

    // Color 1 & 2 Pattern
    SPARKLE_1_ON_2(1685),
    SPARKLE_2_ON_1(1695),
    COLOR_GRADIENT_1_AND_2(1705),
    COLOR_BPM_1_AND_2(1715),
    BLEND_1_TO_2(1725),
    BLEND_1_AND_2(1735),
    COLOR_NO_BLEND_1_AND_2(1745),
    COLOR_TWINKLES_1_AND_2(1755),
    COLOR_WAVES_1_AND_2(1765),
    COLOR_SINELON_1_AND_2(1775),

    // Solid Colors
    HOT_PINK(1785),
    DARK_RED(1795),
    RED(1805),
    RED_ORANGE(1815),
    ORANGE(1825),
    GOLD(1835),
    YELLOW(1845),
    LAWN_GREEN(1855),
    LIME(1865),
    DARK_GREEN(1875),
    GREEN(1885),
    BLUE_GREEN(1895),
    AQUA(1905),
    SKY_BLUE(1915),
    DARK_BLUE(1925),
    BLUE(1935),
    BLUE_VIOLET(1945),
    VIOLET(1955),
    WHITE(1965),
    GRAY(1975),
    DARK_GRAY(1985),
    BLACK(1995);

    private final int pulse;

    BlinkinLEDPattern(int pulse) {
        this.pulse = pulse;
    }

    public int getPulse() {
        return pulse;
    }

    @Override
    public String toString() {
        return name() + " (" + pulse + "us)";
    }
}

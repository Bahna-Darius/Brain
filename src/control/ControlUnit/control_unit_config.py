
class ControlUnitConfig:
    # ===== Speed policy =====
    BASE_SPEED = 300        # normal lane following
    CURVE_SPEED = 100       # slower in sharp turns
    STOP_SPEED = 0

    # ===== Steering =====
    MAX_LANE_STEER = 250

    # ===== Lane confidence =====
    MIN_LANE_CONFIDENCE = 0.3

    # ===== Loop =====
    LOOP_HZ = 20

    CTX_TIMEOUT_S = 0.3
    CURVE_STEER_THRESHOLD = 150

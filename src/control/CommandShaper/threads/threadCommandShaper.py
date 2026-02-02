# Copyright (c) 2026, NovaVision
# All rights reserved.

import time
import threading

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import (
    SafetyOverride,

    # MANUAL (Dashboard)
    SpeedMotor,
    SteerMotor,

    # AUTO (ControlUnit)  ✅ NovaVision
    DesiredSpeed,
    DesiredSteer,

    # Output to SerialHandler
    ShapedSpeedMotor,
    ShapedSteerMotor,
)

from src.control.CommandShaper.command_shaper_config import CommandShaperConfig as CFG


################ NovaVision 26.01.2026 ###############
# Context:
# - CommandShaper shapes commands (ramps/clamps) before SerialHandler.
# - AUTO path:
#     ControlUnit -> DesiredSpeed/DesiredSteer -> CommandShaper -> ShapedSpeed/Steer -> threadWrite
# - MANUAL path stays intact:
#     Dashboard SpeedMotor/SteerMotor can still be used as fallback when AUTO inputs missing.
# - SafetyOverride is hard authority.
#####################################################


class threadCommandShaper(threading.Thread):
    def __init__(self, queueList, stop_event, debugging=False):
        super().__init__()
        self.queueList = queueList
        self.stop_event = stop_event
        self.debugging = bool(debugging)  # ✅ NovaVision debugger flag

        # Desired (raw inputs)
        self.desired_speed = 0
        self.desired_steer = 0
        self.last_input_time = time.time()

        # Output (shaped)
        self.out_speed = 0
        self.out_steer = 0

        # Track last published (optional)
        self._last_pub_speed = None
        self._last_pub_steer = None

        # Safety override
        self.safety_override = None
        self.safety_expiry = 0.0

        # ---------------- Subscribers ----------------
        # MANUAL inputs (Dashboard)
        self.manualSpeedSub = messageHandlerSubscriber(self.queueList, SpeedMotor, "lastOnly", True)
        self.manualSteerSub = messageHandlerSubscriber(self.queueList, SteerMotor, "lastOnly", True)

        # AUTO inputs (ControlUnit) ✅ NovaVision
        self.autoSpeedSub = messageHandlerSubscriber(self.queueList, DesiredSpeed, "lastOnly", True)
        self.autoSteerSub = messageHandlerSubscriber(self.queueList, DesiredSteer, "lastOnly", True)

        # Safety
        self.safetySub = messageHandlerSubscriber(self.queueList, SafetyOverride, "lastOnly", True)

        # ---------------- Senders ----------------
        self.speedSender = messageHandlerSender(self.queueList, ShapedSpeedMotor)
        self.steerSender = messageHandlerSender(self.queueList, ShapedSteerMotor)

    def _clamp(self, val, vmin, vmax):
        return max(vmin, min(vmax, val))

    def _ramp(self, current, target, max_step):
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + max_step * (1 if delta > 0 else -1)

    def _apply_timeout(self):
        if time.time() - self.last_input_time > CFG.INPUT_TIMEOUT_S:
            self.desired_speed = CFG.SAFE_SPEED
            self.desired_steer = CFG.SAFE_STEER

    def run(self):
        period = 1.0 / CFG.PUBLISH_HZ
        tick = 0

        while not self.stop_event.is_set():
            start = time.time()
            tick += 1

            # ==========================================================
            # ✅ NovaVision: Prefer AUTO inputs, fallback to MANUAL
            # ==========================================================
            speedMsg = self.autoSpeedSub.receive()
            steerMsg = self.autoSteerSub.receive()

            if speedMsg is None:
                speedMsg = self.manualSpeedSub.receive()
            if steerMsg is None:
                steerMsg = self.manualSteerSub.receive()

            if speedMsg is not None:
                self.desired_speed = int(speedMsg)
                self.last_input_time = time.time()

            if steerMsg is not None:
                self.desired_steer = int(steerMsg)
                self.last_input_time = time.time()

            # ---- Safety override ----
            safetyMsg = self.safetySub.receive()
            if safetyMsg is not None:
                if isinstance(safetyMsg, dict):
                    self.safety_override = safetyMsg
                    self.safety_expiry = time.time() + (float(safetyMsg.get("ttl_ms", 0)) / 1000.0)
                else:
                    self.safety_override = None
                    self.safety_expiry = 0.0

            # ---- Input timeout ----
            self._apply_timeout()

            # ---- Base clamp ----
            tgt_speed = self._clamp(self.desired_speed, CFG.MIN_SPEED, CFG.MAX_SPEED)
            tgt_steer = self._clamp(self.desired_steer, CFG.MIN_STEER, CFG.MAX_STEER)

            # ---- Safety override (hard authority) ----
            now = time.time()
            if self.safety_override and now < self.safety_expiry:
                if bool(self.safety_override.get("stop", False)):
                    self.out_speed = 0
                    self.out_steer = 0
                    tgt_speed = 0
                    tgt_steer = 0
                else:
                    max_speed = self.safety_override.get("max_speed")
                    if max_speed is not None:
                        tgt_speed = self._clamp(tgt_speed, -abs(int(max_speed)), abs(int(max_speed)))

                    max_steer = self.safety_override.get("max_steer")
                    if max_steer is not None:
                        tgt_steer = self._clamp(tgt_steer, -abs(int(max_steer)), abs(int(max_steer)))

            elif self.safety_override and now >= self.safety_expiry:
                self.safety_override = None

            # ---- Ramp ----
            self.out_speed = int(self._ramp(self.out_speed, tgt_speed, CFG.MAX_SPEED_STEP))
            self.out_steer = int(self._ramp(self.out_steer, tgt_steer, CFG.MAX_STEER_STEP))

            ################ NovaVision 26.01.2026 ###############
            # Debugger:
            # - Confirms the pipe: Desired -> Shaped -> SerialHandler.
            #####################################################
            if self.debugging and (tick % 20 == 0):
                s = self.safety_override if self.safety_override else {}
                print(f"[Shaper] tick={tick} desired=({self.desired_speed},{self.desired_steer}) "
                      f"out=({self.out_speed},{self.out_steer}) stop={s.get('stop', False)}")

           # ---- Publish (send every cycle to keep SerialHandler from timing out) ----
            self.speedSender.send(self.out_speed)
            self.steerSender.send(self.out_steer)

            self._last_pub_speed = self.out_speed
            self._last_pub_steer = self.out_steer

            # ---- Timing ----
            elapsed = time.time() - start
            time.sleep(max(0.0, period - elapsed))

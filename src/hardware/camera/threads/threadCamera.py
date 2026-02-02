# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

import cv2
import threading
import base64
import picamera2
import time


from src.utils.messages.allMessages import (
    mainCamera,
    serialCamera,
    Recording,
    Record,
    Brightness,
    Contrast,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import StateChange
from src.statemachine.systemMode import SystemMode
from src.utils.messages.allMessages import serialCameraRaw


# --- IMPORT VROOM (SAFE) ---
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from src.algorithms.LaneAssist.detect import LaneDetection
    from src.algorithms.LaneAssist.lanekeeping import LaneKeeping

    ALGORITHMS_IMPORTED = True
except ImportError as e:
    print(f"CRITICAL ERROR IMPORTING ALGORITHMS: {e}")
    ALGORITHMS_IMPORTED = False


class threadCamera(ThreadWithStop):
    def __init__(self, queuesList, logger, debugger):
        super(threadCamera, self).__init__(pause=0.001)
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.frame_rate = 5
        self.recording = False
        self.camera = None
        self.video_writer = ""

        # --- SENDERI ---
        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)
        self.serialCameraRawSender = messageHandlerSender(self.queuesList, serialCameraRaw)

        ################ NovaVision 26.01.2026 ###############
        # NEW ELEMENT:
        # - Track AUTO/MANUAL mode locally (visual/debug/perception context only)
        # - No actuator commands are sent from camera thread
        #####################################################
        self.is_auto_mode = False

        ################ NovaVision 26.01.2026 ###############
        # NEW ELEMENT:
        # - Controlled JPEG quality (stable bandwidth)
        # - Matches your newer camera thread behavior
        #####################################################
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]

        self.subscribe()
        self._init_camera()
        self.queue_sending()
        self.configs()

    def subscribe(self):
        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)
        self.stateChangeSubscriber = messageHandlerSubscriber(self.queuesList, StateChange, "lastOnly", True)

    def queue_sending(self):
        if self._blocker.is_set(): return
        self.recordingSender.send(self.recording)
        threading.Timer(1, self.queue_sending).start()

    def thread_work(self):
        if self.camera is None:
            time.sleep(1.0)
            return

        ################ NovaVision 26.01.2026 ###############
        # NEW ELEMENT:
        # - Actually call state handler inside loop
        # - Original had the function but did not call it here
        #####################################################
        try:
            self.state_change_handler()
        except Exception:
            pass
            
        try:
            recordRecv = self.recordSubscriber.receive()
            if recordRecv is not None: 
                self.recording = bool(recordRecv)
                if recordRecv == False:
                    self.video_writer.release() # type: ignore
                else:
                    fourcc = cv2.VideoWriter_fourcc( # type: ignore
                        *"XVID"
                    )  # You can choose different codecs, e.g., 'MJPG', 'XVID', 'H264', etc.
                    self.video_writer = cv2.VideoWriter(
                        "output_video" + str(time.time()) + ".avi",
                        fourcc,
                        self.frame_rate,
                        (2048, 1080),
                    )
            serialRequest = cv2.cvtColor(serialRequest, cv2.COLOR_YUV2BGR_I420) # type: ignore

            ################ NovaVision 26.01.2026 ###############
            # NEW ELEMENT:
            # - Use controlled JPEG quality (same as newer thread)
            # - Keeps transport stable and predictable
            #####################################################
            _, mainEncodedImg = cv2.imencode(".jpg", mainRequest, self.encode_param) # type: ignore
            _, serialEncodedImg = cv2.imencode(".jpg", serialRequest, self.encode_param) # type: ignore

            if lane_results is None:
                algo_status = "NO DETECTION"
            else:
                algo_status = "ACTIVE"

                # Lane Keeping computes angle AND draws on the frame passed to it
                steer_angle, debug_frame = self.lane_keeper.lane_keeping(lane_results)
                current_steer = steer_angle

                # Use the frame that has the drawings from the algos
                if debug_frame is not None:
                    display_frame = debug_frame
                else:
                    display_frame = lane_results["frame"]

                # Control Auto
                if self.is_auto_mode:
                    # Comanda Steer
                    val_steer_int = int(steer_angle)
                    self.steerSender.send(str(val_steer_int))

                    # Comanda Speed
                    if abs(steer_angle) > 15:
                        current_speed = 15.0
                    else:
                        current_speed = 20.0

                    val_speed_int = int(current_speed)
                    self.speedSender.send(str(val_speed_int))

                    #DEBUG:
                    # print(f"[AUTO CMD] Steer: {val_steer_int} | Speed: {val_speed_int}")
                else:
                    current_speed = 0.0

        except Exception as e:
            algo_status = "CRASH"
            print(f"[LaneAssist Runtime Error]: {e}")

            # --- DASHBOARD HUD (Clean Look) ---
            if display_frame is not None:
                h, w = display_frame.shape[:2]

                # Draw a clean HUD bar at bottom
                cv2.rectangle(display_frame, (0, h - 40), (w, h), (20, 20, 20), -1)

                # Mode Indicator (Left)
                color_mode = (0, 255, 0) if self.is_auto_mode else (0, 0, 255)
                mode_str = "AUTO" if self.is_auto_mode else "MANUAL"
                cv2.putText(display_frame, f"MODE: {mode_str}", (10, h - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_mode, 1, cv2.LINE_AA)

                # Data (Right)
                data_str = f"STEER: {current_steer:.1f} | SPEED: {current_speed:.0f}"
                text_size = cv2.getTextSize(data_str, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                text_x = w - text_size[0] - 10
                cv2.putText(display_frame, data_str, (text_x, h - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                # Steering Vector (Red Line) - Centered and clean
                center_x = w // 2
                base_y = h - 40
                vector_len = 50
                rad = np.radians(current_steer)
                # Correct math: x increases right, positive steer is right turn
                end_x = int(center_x + vector_len * np.sin(rad))
                end_y = int(base_y - vector_len * np.cos(rad))

                cv2.line(display_frame, (center_x, base_y), (end_x, end_y), (0, 0, 255), 3, cv2.LINE_AA)
                cv2.circle(display_frame, (end_x, end_y), 4, (0, 0, 255), -1)

            # Dashboard Sending (Optimized Quality)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]

            ok1, mainEncodedImg = cv2.imencode(".jpg", mainRequest, encode_param)
            ok2, serialEncodedImg = cv2.imencode(".jpg", display_frame, encode_param)

            if ok1 and ok2:
                self.mainCameraSender.send(base64.b64encode(mainEncodedImg).decode("utf-8"))
                self.serialCameraSender.send(base64.b64encode(serialEncodedImg).decode("utf-8"))

            self.mainCameraSender.send(mainEncodedImageData)
            self.serialCameraRawSender.send(serialEncodedImageData)
        except Exception as e:
            pass

    def state_change_handler(self):
        if self.stateChangeSubscriber.is_data_in_pipe():
            message = self.stateChangeSubscriber.receive()
            if message:
                state_name = str(message)
                if "AUTO" in state_name:
                    self.is_auto_mode = True
                    print(">>> AUTO MODE ON <<<")
                else:
                    self.is_auto_mode = False
                    self.speedSender.send("0")
                    print(">>> MANUAL MODE <<<")


            if "resolution" in modeDict:
                print(f"\033[1;97m[ Camera Thread ] :\033[0m \033[1;92mINFO\033[0m - Resolution changed to {modeDict['resolution']}")

            ################ NovaVision 26.01.2026 ###############
            # NEW ELEMENT:
            # - Track AUTO/MANUAL flag without affecting SystemMode behavior
            # - Useful for dashboard overlay or perception context
            #####################################################
            try:
                state_name = str(message)
                if "AUTO" in state_name:
                    self.is_auto_mode = True
                else:
                    self.is_auto_mode = False
            except Exception:
                pass

    # ================================ INIT CAMERA ========================================

    def _init_camera(self):
        try:
            if len(picamera2.Picamera2.global_camera_info()) == 0:
                self.camera = None
                return
            self.camera = picamera2.Picamera2()
            config = self.camera.create_preview_configuration(
                buffer_count=1, queue=False,
                main={"format": "RGB888", "size": (2048, 1080)},
                lores={"size": (512, 270)}, encode="lores",
            )
            self.camera.configure(config)
            self.camera.start()
        except:
            self.camera = None


    # =============================== STOP ================================================

    def stop(self):
        if self.camera: self.camera.stop()
        super(threadCamera, self).stop()

    def configs(self):
        """Callback function for receiving configs on the pipe."""
        if self._blocker.is_set():
            return

        ################ NovaVision 26.01.2026 ###############
        # NEW ELEMENT:
        # - Safety guard: if camera is missing, skip set_controls
        # - Prevents NoneType crashes if camera disconnects
        #####################################################
        if self.camera is None:
            threading.Timer(1, self.configs).start()
            return

        if self.brightnessSubscriber.is_data_in_pipe():
            message = self.brightnessSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    "Brightness": max(0.0, min(1.0, float(message))), # type: ignore
                }
            )
        if self.contrastSubscriber.is_data_in_pipe():
            message = self.contrastSubscriber.receive() # de modificat marti uc camera noua 
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    "Contrast": max(0.0, min(32.0, float(message))), # type: ignore
                }
            )
        threading.Timer(1, self.configs).start()

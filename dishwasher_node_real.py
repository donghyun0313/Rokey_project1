#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Merged Node: dishwasher_node2 + sss_sh logic
# UI 연동 유지 + sss_sh의 모션 로직 적용

import time
import threading
import traceback
from typing import Optional, Tuple
import os

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

import DR_init
import pyrebase
from firebase_admin import credentials
from dsr_msgs2.srv import GetRobotMode, SetRobotMode

# =========================================================
# 기본 설정 (sss_sh 기반 상수로 통합)
# =========================================================
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

# 속도/가속도 설정
VEL = 60
ACC = 60
FASTER_VEL = 90
FASTER_ACC = 90

# IO 설정 (sss_sh 기준)
DO_CLOSE = 1
DO_OPEN  = 2
DO_WASH  = 3
DO_BRUSH = 4

# MANUAL 모드 값
MANUAL_MODE = 1

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# =========================================================
# Firebase Init
# =========================================================
key_path = os.environ.get("FIREBASE_KEY_PATH", "/home/rokey/cobot_ws/src/cobot1/cobot1/firebase/key/my.json")
try:
    cred = credentials.Certificate(key_path)
except Exception:
    pass # 이미 초기화되었거나 경로 문제시 무시 (로컬 테스트용)

FIREBASE_CONFIG = {

}

firebase = pyrebase.initialize_app(FIREBASE_CONFIG)
rtdb = firebase.database()

def now_ms() -> int:
    return int(time.time() * 1000)

def upload_current_state(state: str, step: str, progress: int, error: str = ""):
    payload = {
        "state": state,
        "step": step,
        "progress": int(progress),
        "error": error,
        "updated_at": now_ms(),
    }
    try:
        rtdb.child("robots").child(ROBOT_ID).child("current").set(payload)
    except Exception as e:
        print("UPLOAD current ERROR:", repr(e))

# =========================================================
# 유틸리티 함수 (sss_sh + dishwasher_node2 통합)
# =========================================================
def _to_list(posj_val):
    if isinstance(posj_val, (list, tuple)): return list(posj_val)
    return list(posj_val)

def _add_z(p6, dz_mm):
    """posx용 6D 리스트에서 z만 +dz 적용"""
    return [p6[0], p6[1], p6[2] + dz_mm, p6[3], p6[4], p6[5]]

# =========================================================
# 그리퍼 제어 (sss_sh 로직 적용)
# =========================================================
def gripper_open():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_OPEN, 1); set_digital_output(DO_CLOSE, 0)
    set_digital_output(DO_WASH, 0); set_digital_output(DO_BRUSH, 0)
    wait(0.3)

def gripper_close():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_OPEN, 0); set_digital_output(DO_CLOSE, 1)
    set_digital_output(DO_WASH, 0); set_digital_output(DO_BRUSH, 0)
    wait(0.3)

def gripper_wash(): 
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_WASH, 1); set_digital_output(DO_OPEN, 0); set_digital_output(DO_CLOSE, 0)
    wait(0.3)

def gripper_width_1mm():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 1); set_digital_output(2, 0); set_digital_output(3, 0); wait(0.8)

def gripper_width_37mm():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 0); set_digital_output(2, 1); set_digital_output(3, 0); wait(0.8)

def gripper_width_55mm():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 0); set_digital_output(2, 0); set_digital_output(3, 1); wait(0.8)

def rotate_j6_to(target_deg, jvel=60, jacc=120):
    from DSR_ROBOT2 import get_current_posj, movej, wait
    j = _to_list(get_current_posj())
    j[5] = float(target_deg)
    movej(j, vel=jvel, acc=jacc)
    wait(0.2)

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

# =========================================================
# Runtime Control (상태 관리)
# =========================================================
class RuntimeControl:
    def __init__(self):
        self._lock = threading.Lock()
        self.running = False
        self.paused = False
        self.stop_requested = False

        self.last_step = "PICK"
        self.last_progress = 0

    def _set_last(self, step: str, progress: int):
        with self._lock:
            self.last_step = step
            self.last_progress = int(progress)

    def get_last(self) -> Tuple[str, int]:
        with self._lock:
            return self.last_step, self.last_progress

    def start(self):
        with self._lock:
            if self.running: return False, "Already running"
            self.running = True
            self.paused = False
            self.stop_requested = False
            return True, "START accepted"

    def pause(self):
        with self._lock:
            if not self.running: return False, "Not running"
            if self.paused: return False, "Already paused"
            self.paused = True
            return True, "PAUSE accepted"
    
    def resume(self):
        with self._lock:
            if not self.running: return False, "Not running"
            if not self.paused: return False, "Not paused"
            self.paused = False
            return True, "RESUME accepted"

    def stop(self):
        with self._lock:
            if not self.running: return False, "Not running"
            self.stop_requested = True
            self.paused = False
            return True, "STOP accepted"

    def reset(self):
        with self._lock:
            self.running = False
            self.paused = False
            self.stop_requested = False
            self.last_step = "PICK"
            self.last_progress = 0
            return True, "RESET accepted"

    def snapshot(self):
        with self._lock:
            return self.running, self.paused, self.stop_requested

    def clear_after_run(self):
        with self._lock:
            self.running = False
            self.paused = False
            self.stop_requested = False

    def checkpoint(self, step_hint: str):
        # 멈춤/일시정지 체크 로직
        _, _, stop_req = self.snapshot()
        if stop_req:
            raise RuntimeError("STOP_REQUESTED")

        while True:
            _, paused, stop_req = self.snapshot()
            if stop_req:
                raise RuntimeError("STOP_REQUESTED")
            if not paused:
                return

            last_step, last_prog = self.get_last()
            # PAUSE 상태를 DB에 알림
            upload_current_state("PAUSED", last_step, last_prog)
            time.sleep(1.0)

# =========================================================
# 공정별 함수 (sss_sh 로직을 분할 적용)
# =========================================================

# Helper for movel wrapping
def movel_call(p, ref=None, mod=None, radius=0.0):
    from DSR_ROBOT2 import movel
    kwargs = dict(vel=VEL, acc=ACC, radius=radius)
    if ref is not None: kwargs["ref"] = ref
    if mod is not None: kwargs["mod"] = mod
    return movel(p, **kwargs)

def set_step(ctrl: RuntimeControl, state: str, step: str, progress: int, error: str = ""):
    ctrl._set_last(step, progress)
    upload_current_state(state, step, progress, error=error)

def pick_tray(ctrl: RuntimeControl):
    """ sss_sh [STEP 0] ~ [STEP 1] """
    from DSR_ROBOT2 import (
        movejx, movel, posx, wait, get_current_posx,
        task_compliance_ctrl, release_compliance_ctrl, set_desired_force, set_ref_coord
    )
    
    set_step(ctrl, "공정 시작", "PICK", 1)
    ctrl.checkpoint("PICK")
    set_ref_coord(0) # Base 기준

    # [STEP 0] 홈 포지션
    home_x = posx([367.28, 8.93, 422.97, 4.37, 179.98, 3.97])
    movejx(home_x, vel=VEL, acc=ACC)
    ctrl.checkpoint("PICK")
    wait(0.5)

    # [STEP 1] 식판 집기 (Pick Up)
    tray_approach = posx([-289.24, -141, 544.47, 168.93, 179.38, -9.83])
    movejx(tray_approach, vel=VEL, acc=ACC, time=5.0)
    ctrl.checkpoint("PICK")
    
    gripper_open()
    ctrl.checkpoint("PICK")

    task_compliance_ctrl()
    set_desired_force([5, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0])

    movel(tray_approach, vel=VEL, acc=VEL) # sss_sh 로직 유지
    ctrl.checkpoint("PICK")

    release_compliance_ctrl()

    cp_pick = get_current_posx()[0]
    tray_down = posx([cp_pick[0], cp_pick[1], cp_pick[2] - 290, cp_pick[3], cp_pick[4], cp_pick[5]])
    
    movel(tray_down, vel=VEL, acc=ACC)
    ctrl.checkpoint("PICK")
    
    gripper_close()
    ctrl.checkpoint("PICK")

    movel(cp_pick, vel=VEL, acc=ACC)
    ctrl.checkpoint("PICK")

    set_step(ctrl, "PICK 완료", "PICK", 20)

def trash_out(ctrl: RuntimeControl):
    """ sss_sh [STEP 2] """
    from DSR_ROBOT2 import movel, movej, posx, get_current_posj, wait

    set_step(ctrl, "잔반 처리 시작", "TURBID", 21)
    ctrl.checkpoint("TURBID")

    P_TRASH_UP = posx([-19.01, -374.20, 540.00, 80.62, -174.96, -7.70])
    P_TRASH_DOWN_SAFE = posx([-54.45, -364.25, 540.00, 87.89, -149.56, -0.96])
    P_TRASH_DOWN = posx([-54.45, -364.25, 252.28, 87.89, -149.56, -0.96])

    movel(P_TRASH_UP, vel=VEL, acc=ACC); ctrl.checkpoint("TURBID")
    movel(P_TRASH_DOWN_SAFE, vel=VEL, acc=ACC); ctrl.checkpoint("TURBID")
    movel(P_TRASH_DOWN, vel=VEL, acc=ACC); ctrl.checkpoint("TURBID")

    base_j = _to_list(get_current_posj())
    j5_center = base_j[4]
    
    set_step(ctrl, "잔반 털기", "TURBID", 30)
    for _ in range(5):
        ctrl.checkpoint("TURBID")
        j_up = base_j.copy(); j_up[4] = j5_center + 5.0
        j_dn = base_j.copy(); j_dn[4] = j5_center - 5.0
        movej(j_up, vel=VEL, acc=ACC)
        movej(j_dn, vel=VEL, acc=ACC)

    movel(P_TRASH_DOWN_SAFE, vel=VEL, acc=ACC); ctrl.checkpoint("TURBID")
    movel(P_TRASH_UP, vel=VEL, acc=ACC); ctrl.checkpoint("TURBID")

    set_step(ctrl, "잔반 처리 완료", "TURBID", 40)

def hot_water_wash(ctrl: RuntimeControl):
    """ sss_sh [STEP 3] """
    from DSR_ROBOT2 import movel, posx, move_periodic

    set_step(ctrl, "온수 세척 시작", "HOT", 41)
    ctrl.checkpoint("HOT")

    hot_top = posx([297.06, -437.00, 540.00, 40.38, -177.85, -47.41])
    hot_down = posx([291.20, -444.58, 308.34, 35.15, -177.60, -52.77])
    
    movel(hot_top, vel=FASTER_VEL, acc=FASTER_ACC); ctrl.checkpoint("HOT")
    movel(hot_down, vel=VEL, acc=ACC); ctrl.checkpoint("HOT")
    
    move_periodic(amp=[0, 60, 0, 0, 0, 0], period=2.0, atime=0.5, repeat=3, ref=0)
    ctrl.checkpoint("HOT")
    
    move_periodic(amp=[0, 0, 30, 0, 0, 0], period=2.0, atime=0.5, repeat=3, ref=0)
    ctrl.checkpoint("HOT")
    
    movel(hot_top, vel=VEL, acc=ACC); ctrl.checkpoint("HOT")

    set_step(ctrl, "온수 세척 완료", "HOT", 50)

def scrub_wash(ctrl: RuntimeControl):
    """ sss_sh [STEP 4] Complex Logic """
    from DSR_ROBOT2 import (
        posx, movel, movej, set_ref_coord, set_singularity_handling,
        DR_BASE, DR_AVOID, DR_MV_MOD_ABS, DR_MV_MOD_REL, wait
    )

    set_step(ctrl, "솔질 세척 준비", "BRUSH", 51)
    ctrl.checkpoint("BRUSH")

    set_singularity_handling(DR_AVOID)

    # 4-1) 식판 릴리즈 + 안전상승
    set_ref_coord(DR_BASE)

    plate_release_6d = [649.520, -12.250, 428.380, 151.75, -176.51, -28.70]
    plate_release    = posx(plate_release_6d)
    
    brush_up   = posx([585.890, 146.840, 434.800, 163.90, -178.49, 167.17])
    brush_down = posx([591.340, 147.690, 366.960, 164.44, -178.38, 167.61])
    
    # 중간 경로로 이동 후
    plate_up1_6d = posx([594.610, -11.050, 528.980, 168.26, -171.74, -11.87])
    
    movel_call(plate_up1_6d , ref=DR_BASE); ctrl.checkpoint("BRUSH")
    wait(0.2)
    movel_call(plate_release, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    
    gripper_width_55mm()
    wait(0.2)

    plate_release_up = posx(_add_z(plate_release_6d, 100.0))
    movel_call(plate_release_up, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    wait(0.2)

    # 4-2) 브러시 접근
    movel_call(brush_up, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    wait(0.2)
    movel_call(brush_down, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    wait(0.2)

    # 4-3) 브러시 파지
    gripper_width_1mm()
    wait(0.2)

    movel_call(brush_up, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    wait(0.2)

    # 자세 정리
    rotate_j6_to(-82.0, jvel=60, jacc=120)
    ctrl.checkpoint("BRUSH")

    # 4-4) 지그재그 패턴
    set_step(ctrl, "솔질 중", "BRUSH", 60)
    
    REF_USER = 101
    set_ref_coord(REF_USER)

    HOME = posx(367.28, 8.93, 422.97, 4.37, 179.98, 3.97)
    PLATELEFT = posx(478.840, 76.220, 314.870, 178.3, -136.46, -178.26)

    STEP_X = 15.0
    DOWN_Y = 25.0
    REPEAT_N = 6

    def movel_abs(p, ref):
        return movel_call(p, ref=ref, mod=DR_MV_MOD_ABS, radius=0.0)
    
    def movel_rel(ref, dx=0.0, dy=0.0, dz=0.0, drz=0.0):
        dp = posx(dx, dy, dz, 0.0, 0.0, drz)
        return movel_call(dp, ref=ref, mod=DR_MV_MOD_REL, radius=0.0)

    movel_abs(HOME, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    movel_abs(PLATELEFT, ref=DR_BASE); ctrl.checkpoint("BRUSH")

    def col_pattern_x(x_sign, repeat_n=REPEAT_N):
        dx = STEP_X * x_sign
        for _ in range(repeat_n):
            ctrl.checkpoint("BRUSH") # 패턴 내부 체크포인트
            movel_rel(REF_USER, dx=dx, drz=+30.0)
            movel_rel(REF_USER, dx=dx, drz=-30.0)

    # 1~5열
    col_pattern_x(+1); movel_rel(REF_USER, dy=-DOWN_Y)
    col_pattern_x(-1); movel_rel(REF_USER, dy=-DOWN_Y)
    col_pattern_x(+1); movel_rel(REF_USER, dy=-DOWN_Y)
    col_pattern_x(-1); movel_rel(REF_USER, dy=-DOWN_Y)
    col_pattern_x(+1)

    # 4-5) 종료: base 복귀
    set_ref_coord(DR_BASE)
    movel_call(HOME, ref=DR_BASE); ctrl.checkpoint("BRUSH")

    # 브러시 내려놓고 식판 다시 집기 로직
    set_step(ctrl, "식판 회수", "BRUSH", 70)

    JReady = [0, 0, 90, 0, 90, 0]
    
    # Coordinates for grabbing plate again
    brush_up   = posx(585.890, 146.840, 434.800, 163.90, -178.49, 167.17)
    brush_down = posx(591.340, 147.690, 366.960, 164.44, -178.38, 167.61)

    plate_grab_down = posx(694.94, -13.72, 346.44, 125.86, -177.59, 127.44)
    plate_grab_up   = posx(698.13, -14.26, 360.97, 161.29, -172.72, 162.84)
    plate_pull      = posx(549.59, 2.19, 574.29, 167.00, -164.91, 168.13)
    plate_pull_up   = posx(246.73, 3.25, 526.63, 164.10, -170.55, 169.11)

    movej(JReady, vel= VEL, acc=ACC); ctrl.checkpoint("BRUSH")
    movel(HOME, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE); ctrl.checkpoint("BRUSH")

    # 브러시 반납
    movel(brush_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    movel(brush_down, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    gripper_width_37mm()
    wait(0.2)
    movel(brush_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE); ctrl.checkpoint("BRUSH")

    # 식판 잡으러 가기
    movel(plate_grab_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    movel(plate_grab_down, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    gripper_width_1mm()
    wait(0.5)

    movel(plate_grab_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    wait(0.3)
    movel(plate_pull, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE); ctrl.checkpoint("BRUSH")
    movel(plate_pull_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE); ctrl.checkpoint("BRUSH")

    set_step(ctrl, "솔질 완료", "BRUSH", 75)

def final_wash_and_stack(ctrl: RuntimeControl):
    """ sss_sh [STEP 5] & [STEP 6] """
    from DSR_ROBOT2 import movel, posx, move_periodic, get_current_posx, DR_BASE, wait

    # [STEP 5] 냉수 세척
    set_step(ctrl, "냉수 세척 시작", "COLD", 80)
    ctrl.checkpoint("COLD")

    water_top = posx([275.22, 526.54, 458.35, 1.11, -177.15, -177.39])
    movel(water_top, vel=VEL, acc=ACC); ctrl.checkpoint("COLD")
    
    cp_wash = get_current_posx()[0]
    cold_soak = posx([cp_wash[0], cp_wash[1], cp_wash[2] - 281, cp_wash[3], cp_wash[4], cp_wash[5] + 90])
    
    movel(cold_soak, vel=VEL, acc=ACC); ctrl.checkpoint("COLD")
    
    move_periodic(amp=[60, 60, 0, 0, 0, 0], period=[3.2, 1.6, 0, 0, 0, 0], atime=3.1, repeat=4, ref=DR_BASE)
    ctrl.checkpoint("COLD")
    
    movel(cp_wash, vel=VEL, acc=ACC); ctrl.checkpoint("COLD")

    # [STEP 6] 적재
    set_step(ctrl, "식판 적재 중", "STACK", 90)
    ctrl.checkpoint("STACK")

    on_my_way_mid = posx([-74.630, 429.650, 474.090, 175.36, 177.4, -93.9])
    tray_up2 = posx([-269.990, 132.5, 540.540, 1.5,-178.04, 2.64])
    tray_down2 = posx([-280.75, 133.40, 260.380, 8.29, -178.4, 9.34])
    home_x = posx([367.28, 8.93, 422.97, 4.37, 179.98, 3.97])

    movel(on_my_way_mid, vel=VEL, acc=ACC); ctrl.checkpoint("STACK")
    movel(tray_up2, vel=VEL, acc=ACC); ctrl.checkpoint("STACK")
    movel(tray_down2, vel=VEL, acc=ACC); ctrl.checkpoint("STACK")

    gripper_wash()
    wait(0.5)

    movel(tray_up2, vel=VEL, acc=ACC); ctrl.checkpoint("STACK")
    movel(home_x, vel=VEL, acc=ACC); ctrl.checkpoint("STACK")

    set_step(ctrl, "모든 공정 완료", "DONE", 100)

def run_full_sequence(ctrl: RuntimeControl):
    """ 메인 시퀀스 실행부 """
    pick_tray(ctrl)
    trash_out(ctrl)
    hot_water_wash(ctrl)
    scrub_wash(ctrl)
    final_wash_and_stack(ctrl)

# =========================================================
# Firebase Listener Thread
# =========================================================
class FirebaseCommandListener(threading.Thread):
    def __init__(self, ctrl: RuntimeControl):
        super().__init__(daemon=True)
        self.ctrl = ctrl
        self.running = True

    def run(self):
        print(">>> [Firebase] Command Listener Started...")
        while self.running and rclpy.ok():
            try:
                cmd_ref = rtdb.child("robots").child(ROBOT_ID).child("command")
                data = cmd_ref.get().val()

                if data and isinstance(data, dict) and data.get("status") == "NEW":
                    cmd_str = str(data.get("cmd", "")).strip().upper()
                    print(f"\n======== [Firebase] Command: {cmd_str} ========")

                    ok, msg = False, "Unknown Command"
                    
                    if cmd_str == "START":
                        ok, msg = self.ctrl.start()
                    elif cmd_str == "PAUSE":
                        ok, msg = self.ctrl.pause()
                    elif cmd_str == "RESUME":
                        ok, msg = self.ctrl.resume()
                    elif cmd_str == "RESET":
                        ok, msg = self.ctrl.reset()
                    elif cmd_str == "STOP":
                        ok, msg = self.ctrl.stop()

                    cmd_ref.update({
                        "status": "DONE",
                        "result_ok": ok,
                        "result_msg": msg,
                        "processed_at": now_ms()
                    })
                    print(f">>> [Firebase] Result: {msg}")
                    print("==============================================\n")
            except Exception as e:
                print(f"[Firebase Listener Error] {e}")
            time.sleep(1.0) 

# =========================================================
# ROS Service Node
# =========================================================
class DishwasherServiceNode(Node):
    def __init__(self, ctrl: RuntimeControl):
        super().__init__("dishwasher_service_node", namespace=ROBOT_ID)
        self.ctrl = ctrl
        self.cb_group = ReentrantCallbackGroup()

        # Services
        self.srv_start = self.create_service(Trigger, "dishwasher/start", self.on_start, callback_group=self.cb_group)
        self.srv_pause = self.create_service(Trigger, "dishwasher/pause", self.on_pause, callback_group=self.cb_group)
        self.srv_stop  = self.create_service(Trigger, "dishwasher/stop",  self.on_stop,  callback_group=self.cb_group)
        self.srv_reset = self.create_service(Trigger, "dishwasher/reset", self.on_reset, callback_group=self.cb_group)
        self.srv_resume = self.create_service(Trigger, "dishwasher/resume", self.on_resume, callback_group=self.cb_group)

        # Clients (for Mode Check)
        self.cli_get_mode = self.create_client(GetRobotMode, "system/get_robot_mode", callback_group=self.cb_group)
        self.cli_set_mode = self.create_client(SetRobotMode, "system/set_robot_mode", callback_group=self.cb_group)

        self.get_logger().info("Dishwasher Services Ready.")

    # Callbacks
    def on_start(self, request, response):
        ok, msg = self.ctrl.start()
        response.success = bool(ok); response.message = msg
        if ok: upload_current_state("START_RECEIVED", "PICK", 0)
        return response

    def on_pause(self, request, response):
        ok, msg = self.ctrl.pause()
        response.success = bool(ok); response.message = msg
        if ok:
            step, prog = self.ctrl.get_last()
            upload_current_state("PAUSE_RECEIVED", step, prog)
        return response

    def on_resume(self, request, response):
        ok, msg = self.ctrl.resume()
        response.success = bool(ok); response.message = msg
        return response

    def on_stop(self, request, response):
        ok, msg = self.ctrl.stop()
        response.success = bool(ok); response.message = msg
        if ok:
            step, prog = self.ctrl.get_last()
            upload_current_state("STOP_RECEIVED", step, prog)
        return response

    def on_reset(self, request, response):
        self.ctrl.reset()
        upload_current_state("준비", "PICK", 0, error="")
        response.success = True; response.message = "RESET accepted"
        return response

# =========================================================
# Main Loop
# =========================================================
def main(args=None):
    rclpy.init(args=args)

    ctrl = RuntimeControl()
    node: Optional[DishwasherServiceNode] = None
    executor: Optional[MultiThreadedExecutor] = None
    fb_listener: Optional[FirebaseCommandListener] = None 

    try:
        node = DishwasherServiceNode(ctrl)

        DR_init.__dsr__node = node
        DR_init.__dsr__srv_name_prefix   = f"/{ROBOT_ID}"
        DR_init.__dsr__topic_name_prefix = f"/{ROBOT_ID}"

        # 1. ROS2 Spin Thread
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        print(">>> [System] ROS2 Spin Thread Started")

        # 2. Clear Stale DB Commands
        print(">>> [System] Cleaning up stale DB commands...")
        try:
            rtdb.child("robots").child(ROBOT_ID).child("command").update({
                "status": "DONE",
                "result_msg": "System Restarted (Cleared)"
            })
        except Exception: pass

        # 3. Firebase Listener Start
        fb_listener = FirebaseCommandListener(ctrl)
        fb_listener.start()
        
        # 4. Robot Init
        print(">>> [System] Initializing Robot settings...")
        try:
            initialize_robot()
        except Exception as e:
            print("WARN: initialize_robot failed (But proceeding):", repr(e))

        upload_current_state("준비", "PICK", 0)
        print(">>> [System] Waiting for Web Command...")

        while rclpy.ok():
            running, _, _ = ctrl.snapshot()

            if running:
                try:
                    print(">>> [System] Starting Process (Integrated)")
                    upload_current_state("RUNNING", "PICK", 1)
                    
                    # sss_sh의 모션 시퀀스 실행
                    run_full_sequence(ctrl)
                    
                    upload_current_state("ALL_DONE", "DONE", 100)
                    print(">>> [System] Process Complete. Back to HOME.")

                except Exception as e:
                    is_stop = ("STOP_REQUESTED" in str(e))
                    step, prog = ctrl.get_last()
                    upload_current_state(
                        "STOPPED" if is_stop else "ERROR",
                        step,
                        prog if is_stop else 0,
                        error=str(e),
                    )
                    print(f">>> [Run Error] {e}")
                    traceback.print_exc()
                finally:
                    ctrl.clear_after_run()
                    upload_current_state("준비", "PICK", 0)

            time.sleep(0.1)

    finally:
        try:
            if fb_listener:
                fb_listener.running = False
                fb_listener.join(timeout=0.5) 
        except Exception: pass
        try:
            if executor is not None and node is not None:
                executor.remove_node(node)
        except Exception: pass
        try:
            if node is not None:
                node.destroy_node()
        except Exception: pass
        try:
            rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()
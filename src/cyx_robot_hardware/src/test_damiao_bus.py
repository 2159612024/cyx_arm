#!/usr/bin/env python3
"""Bench-test six daMiao motors connected to one serial USB2CAN adapter.

The script first reads identity/control parameters from every motor.  Only when
all six reply does it offer a simultaneous, low-speed forward test.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import serial


MOTOR_LAYOUT = (
    (1, 0x01, 0x11, "DM4340"),
    (2, 0x02, 0x12, "DM4340"),
    (3, 0x03, 0x13, "DM4340"),
    (4, 0x04, 0x14, "DM4310"),
    (5, 0x05, 0x15, "DM4310"),
    (6, 0x06, 0x16, "DM4310"),
)


def default_dm_can_dir() -> Path:
    return (
        Path(__file__).resolve().parents[4]
        / "dm_reference"
        / "motor-control-routine-master"
        / "Python例程"
        / "u2can"
    )


def load_driver(driver_dir: Path):
    driver = driver_dir.resolve()
    if not (driver / "DM_CAN.py").is_file():
        raise FileNotFoundError(f"未找到厂商驱动: {driver / 'DM_CAN.py'}")
    sys.path.insert(0, str(driver))
    from DM_CAN import (  # pylint: disable=import-outside-toplevel
        Control_Type,
        DM_Motor_Type,
        DM_variable,
        Motor,
        MotorControl,
    )

    return Control_Type, DM_Motor_Type, DM_variable, Motor, MotorControl


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="串口，如 COM3 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--speed", type=float, default=0.20, help="测试速度 rad/s")
    parser.add_argument("--duration", type=float, default=30.0, help="正转持续秒数")
    parser.add_argument("--samples", type=int, default=3, help="每台电机连通性读取轮数")
    parser.add_argument("--no-motion", action="store_true", help="只检查通信，不转动")
    parser.add_argument("--dm-can-dir", type=Path, default=default_dm_can_dir())
    args = parser.parse_args()
    if not (0.0 < args.speed <= 0.5):
        parser.error("--speed 必须在 (0, 0.5] rad/s")
    if not (0.05 <= args.duration <= 60.0):
        parser.error("--duration 必须在 [0.05, 60.0] s")
    if not (1 <= args.samples <= 20):
        parser.error("--samples 必须在 [1, 20]")
    return args


def make_motors(Motor, motor_types):
    return [
        Motor(getattr(motor_types, model), can_id, master_id)
        for _, can_id, master_id, model in MOTOR_LAYOUT
    ]


def safe_stop(control, motors) -> None:
    for motor in motors:
        try:
            control.control_Vel(motor, 0.0)
        except Exception:
            pass
    for motor in motors:
        try:
            control.disable(motor)
        except Exception:
            pass


def read_and_validate(control, motor, variables, velocity_mode: int, samples: int):
    expected = {
        variables.ESC_ID: motor.SlaveID,
        variables.MST_ID: motor.MasterID,
        variables.CTRL_MODE: velocity_mode,
    }
    values = {}
    for _ in range(samples):
        for variable in expected:
            value = None
            for _attempt in range(5):
                motor.temp_param_dict.pop(variable, None)
                value = control.read_motor_param(motor, variable)
                if value is not None:
                    break
                time.sleep(0.05)
            if value is None:
                raise RuntimeError(
                    f"电机 0x{motor.SlaveID:02X} 读取 {variable.name} 超时"
                )
            if variable in values and value != values[variable]:
                raise RuntimeError(
                    f"电机 0x{motor.SlaveID:02X} 的 {variable.name} 多次读取不一致"
                )
            values[variable] = value
    mismatches = [
        f"{key.name}={values[key]} (应为 {wanted})"
        for key, wanted in expected.items()
        if int(values[key]) != int(wanted)
    ]
    if mismatches:
        raise RuntimeError(
            f"电机 0x{motor.SlaveID:02X} 配置不符: " + ", ".join(mismatches)
        )
    return values


def run_all_motors(control, motors, speed: float, duration: float) -> None:
    enabled = []
    try:
        for motor in motors:
            control.enable(motor)
            enabled.append(motor)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            for motor in motors:
                control.control_Vel(motor, speed)
            time.sleep(0.01)
        for motor in motors:
            control.control_Vel(motor, 0.0)
        time.sleep(0.3)
    finally:
        for motor in enabled:
            try:
                control.control_Vel(motor, 0.0)
            except Exception:
                pass
        for motor in enabled:
            try:
                control.disable(motor)
            except Exception:
                pass


def main() -> int:
    args = parse_args()
    ControlType, MotorType, variables, Motor, MotorControl = load_driver(args.dm_can_dir)
    motors = make_motors(Motor, MotorType)
    serial_device = None
    control = None
    try:
        serial_device = serial.Serial(args.port, args.baud, timeout=0.1)
        control = MotorControl(serial_device)
        for motor in motors:
            control.addMotor(motor)

        print("开始检查 6 台电机；任意一台失败都不会进入运动阶段。")
        for joint, _, _, model in MOTOR_LAYOUT:
            motor = motors[joint - 1]
            values = read_and_validate(
                control, motor, variables, int(ControlType.VEL), args.samples
            )
            print(
                f"[OK] J{joint} {model}: CAN=0x{motor.SlaveID:02X}, "
                f"MASTER=0x{int(values[variables.MST_ID]):02X}, "
                f"CTRL_MODE={int(values[variables.CTRL_MODE])}"
            )

        if args.no_motion:
            print("通信检查全部通过；--no-motion 已跳过运动。")
            return 0

        print("\n请确认六台电机均已卸载/可靠固定，转轴周围无人员和线缆，硬件急停可触及。")
        print(
            f"六台电机将同时正转：{args.speed:.3f} rad/s，持续 {args.duration:.1f}s。"
        )
        if input("输入 RUN ALL 开始六电机同时测试: ").strip() != "RUN ALL":
            print("已取消，未使能电机。")
            return 0
        print("六台电机开始同时正转；Ctrl+C 可中止。")
        run_all_motors(control, motors, args.speed, args.duration)
        print("六台电机已停止并失能。")
        if input("六台电机运动是否全部正常？输入 Y 确认: ").strip().upper() != "Y":
            raise RuntimeError("六电机运动未确认正常")
        print("六台电机通信和同时运动测试全部完成。")
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在停止并失能全部电机。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\n测试中止: {exc}", file=sys.stderr)
        return 1
    finally:
        if control is not None:
            safe_stop(control, motors)
        if serial_device is not None and serial_device.is_open:
            serial_device.close()


if __name__ == "__main__":
    raise SystemExit(main())

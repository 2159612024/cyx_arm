#!/usr/bin/env python3
"""Interactively set six daMiao joint zeros, then run a small-angle test."""

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
    parser.add_argument("--angle-deg", type=float, default=3.0, help="测试角度（电机反馈轴）")
    parser.add_argument("--test-speed", type=float, default=0.15, help="位置测试速度 rad/s")
    parser.add_argument("--dm-can-dir", type=Path, default=default_dm_can_dir())
    args = parser.parse_args()
    if not (0.1 <= args.angle_deg <= 5.0):
        parser.error("--angle-deg 必须在 [0.1, 5.0] 度")
    if not (0.02 <= args.test_speed <= 0.3):
        parser.error("--test-speed 必须在 [0.02, 0.3] rad/s")
    return args


def make_motors(Motor, motor_types):
    return [
        Motor(getattr(motor_types, model), can_id, master_id)
        for _, can_id, master_id, model in MOTOR_LAYOUT
    ]


def safe_disable(control, motors) -> None:
    for motor in motors:
        try:
            control.disable(motor)
        except Exception:
            pass


def require_online(control, motor, variables) -> None:
    value = None
    for _attempt in range(5):
        motor.temp_param_dict.pop(variables.MST_ID, None)
        value = control.read_motor_param(motor, variables.MST_ID)
        if value is not None:
            break
        time.sleep(0.05)
    if value is None or int(value) != motor.MasterID:
        raise RuntimeError(
            f"电机 0x{motor.SlaveID:02X} 无响应或 MasterID 不符（读到 {value!r}）"
        )


def refresh(control, motor) -> tuple[float, float, float]:
    # NaN sentinel lets us distinguish a fresh reply from Motor's initial zeros.
    motor.state_q = math.nan
    motor.state_dq = math.nan
    motor.state_tau = math.nan
    for _ in range(5):
        control.refresh_motor_status(motor)
        if all(math.isfinite(x) for x in (motor.getPosition(), motor.getVelocity(), motor.getTorque())):
            return motor.getPosition(), motor.getVelocity(), motor.getTorque()
        time.sleep(0.05)
    raise RuntimeError(f"电机 0x{motor.SlaveID:02X} 状态反馈超时")


def command_position(control, motor, target: float, speed: float, timeout: float = 8.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        control.control_Pos_Vel(motor, target, speed)
        position = motor.getPosition()
        if math.isfinite(position) and abs(position - target) <= math.radians(0.5):
            return
        time.sleep(0.02)
    raise RuntimeError(
        f"J{motor.SlaveID} 未在 {timeout:.1f}s 内到达 {math.degrees(target):.2f}°"
    )


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
        for motor in motors:
            require_online(control, motor, variables)
        print("6 台电机均在线。零点写入会改变电机零点，请逐关节人工确认。")

        for joint, _, _, model in MOTOR_LAYOUT:
            motor = motors[joint - 1]
            control.disable(motor)
            print(f"\nJ{joint} {model} 已发送失能。请人工拖到期望的机械零位。")
            if input(f"位置确认后输入关节编号 {joint}: ").strip() != str(joint):
                raise RuntimeError("用户取消零点设置")
            control.set_zero_position(motor)
            position, velocity, torque = refresh(control, motor)
            if abs(position) > math.radians(1.0):
                raise RuntimeError(
                    f"J{joint} 设置零点后反馈仍为 {math.degrees(position):.2f}°"
                )
            print(
                f"[OK] J{joint} 零点反馈: position={math.degrees(position):.3f}°, "
                f"velocity={velocity:.3f} rad/s, torque={torque:.3f}"
            )

        print("\n全部零点已设置。即将临时切换至位置速度模式并逐轴测试 ±小角度。")
        print("请支撑机械臂、防止重力下坠，确认限位/线缆余量和硬件急停。")
        if input("输入 TEST 开始: ").strip() != "TEST":
            print("已跳过运动测试；电机保持失能。")
            return 0

        angle = math.radians(args.angle_deg)
        for joint, _, _, model in MOTOR_LAYOUT:
            motor = motors[joint - 1]
            if input(f"\n准备测试 J{joint} {model}，输入 J{joint} 继续: ").strip() != f"J{joint}":
                raise RuntimeError("用户取消运动测试")
            if not control.switchControlMode(motor, ControlType.POS_VEL):
                raise RuntimeError(f"J{joint} 切换位置速度模式失败")
            control.enable(motor)
            try:
                command_position(control, motor, +angle, args.test_speed)
                time.sleep(0.3)
                command_position(control, motor, -angle, args.test_speed)
                time.sleep(0.3)
                command_position(control, motor, 0.0, args.test_speed)
            finally:
                control.disable(motor)
            if input(f"J{joint} 运动是否正常？输入 Y 确认: ").strip().upper() != "Y":
                raise RuntimeError(f"J{joint} 未确认正常，停止后续测试")
        print("六个关节零点设置与小角度测试完成；所有电机已失能。")
        print("注意：测试把控制模式改为 POS_VEL，但没有调用保存参数；请按后续控制方案配置。")
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在失能全部电机。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\n操作中止: {exc}", file=sys.stderr)
        return 1
    finally:
        if control is not None:
            safe_disable(control, motors)
        if serial_device is not None and serial_device.is_open:
            serial_device.close()


if __name__ == "__main__":
    raise SystemExit(main())

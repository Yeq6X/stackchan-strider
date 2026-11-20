"""MuJoCo で、無改造のまま曲がれる動かし方を探す。

脚の位相(クランク角)に合わせて頭をうなずかせ、重心を前後に動かす。1 周に 1 往復。
  頭の目標角 = mid + amp × wave(クランク角 − phase)   (0° = 顔が正面、90° = 顔が真上)
歩く速さも位相に合わせて変えられる。
  ヨーサーボの指令 = command × (1 + speed_mod × sin(クランク角 − speed_phase))
もう 1 つの動かし方(wave = "rock"): クランクを rock_center ± rock_half の間で往復させ、折り返すたびに止まって
頭の向きを head_out / head_back に切り替える。進んだ分を戻るので、向きだけが変わることを狙う。

頭の動く部分の質量と重心の位置は測っていない。仮定の値を振って、結果がどれだけ変わるかを見る。
使い方: python walker/sim_turn.py [--seconds 14] [--set phase|sense|speed] [--video 条件番号]
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
from multiprocessing import Pool

import mujoco
import numpy as np

import design as d
import sim_mujoco as sm

SETTLE = 1.0
_models: dict = {}


def _model(friction: float, mass: float, cg_y: float, cg_z: float, backlash: float = 0.0, servo_torque: float = 1.0, servo_speed: float = 1.0, timestep: float | None = None):
    key = (friction, mass, cg_y, cg_z, backlash, servo_torque, servo_speed, timestep)
    if key not in _models:
        xml = sm.build_xml(friction, head=dict(mass=mass, cg_y=cg_y, cg_z=cg_z), export_meshes=False, backlash_deg=backlash, servo_torque=servo_torque, servo_speed=servo_speed)
        if timestep:
            xml = xml.replace('timestep="0.0005"', f'timestep="{timestep}"')
        path = sm.OUT / f"turn_{os.getpid()}.xml"
        path.write_text(xml, encoding="utf-8")
        _models[key] = mujoco.MjModel.from_xml_path(str(path))
        path.unlink()
    return _models[key]


def _heading(quat: np.ndarray) -> float:
    """車体の前(+Y)が向いている方位 [rad]。上から見て反時計回りが正。"""
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, quat)
    forward = rot.reshape(3, 3)[:, 1]
    return math.atan2(-forward[0], forward[1])


def simulate(case: dict) -> dict:
    model = _model(case["friction"], case["mass"], case["cg_y"], case["cg_z"], case.get("backlash", 0.0), case.get("servo_torque", 1.0), case.get("servo_speed", 1.0), case.get("timestep"))
    data = mujoco.MjData(model)
    joint = lambda name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    shaft_q = model.jnt_qposadr[joint("shaft")]
    table_q = model.jnt_qposadr[joint("table")]
    has_backlash = case.get("backlash", 0.0) > 0
    # 実機で読めるのは首(ターンテーブル)の角度だけ。sense = "table" のとき、折り返しの判定にそれを使う
    sense_table = case.get("sense") == "table"
    control_every = max(1, int(round(case.get("control_dt", 0.0) / model.opt.timestep)))
    servo_peak = shaft_peak = 0.0
    table_v = model.jnt_dofadr[joint("table")]
    stall = sm.SERVO_STALL * case.get("servo_torque", 1.0)
    noload = sm.SERVO_NOLOAD * case.get("servo_speed", 1.0)
    current_sum = current_sq_sum = 0.0
    overload_steps = overload_run = overload_longest = samples = 0
    timed = case.get("timed", 0.0)  # > 0 のとき、角度を読まずに、この秒数ごとに折り返す
    timed_started, last_flip = False, 0.0
    flips: list[float] = []
    pitch_q = model.jnt_qposadr[joint("head_pitch")]
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    body_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in ("belly", "head_mass")}
    mid, amp = math.radians(case["mid"]), math.radians(case["amp"])
    phase, speed_phase = math.radians(case["phase"]), math.radians(case["speed_phase"])
    dt = model.opt.timestep
    steps = int(round(case["seconds"] / dt))
    data.qpos[pitch_q] = mid
    target = mid
    renderer, frames = None, []
    if case.get("video"):
        renderer = mujoco.Renderer(model, 720, 960)
    start = None
    heading_prev, heading_sum = _heading(data.qpos[3:7]), 0.0
    path_len, fallen, pitch_error = 0.0, False, 0.0
    rock = case["wave"] == "rock"
    if rock:
        center, half = math.radians(case["rock_center"]), math.radians(case["rock_half"])
        heads = {1: math.radians(case["head_out"]), -1: math.radians(case["head_back"])}
        direction, cycles = 1, 0
        data.qpos[pitch_q] = target = heads[1]
    for step in range(steps):
        time = step * dt
        crank = data.qpos[shaft_q]
        ramp = min(1.0, max(0.0, (time - SETTLE) / 0.5))
        if rock:
            # クランク角が center + direction × half に着いたら止まり、頭を切り替え終えてから逆へ回す
            goal = heads[direction]
            if step % control_every == 0:
                measured = -data.qpos[table_q] if sense_table else crank
                if timed and not timed_started and time >= SETTLE + 0.5:
                    # 往復の始点(center − half)までゆっくり回してから、時間での往復を始める
                    error = (center - half) - crank
                    if abs(error) < math.radians(2):
                        timed_started, last_flip, direction = True, time, 1
                if timed:
                    flip = timed_started and time - last_flip >= timed
                else:
                    flip = (measured - (center + direction * half)) * direction >= 0
                if flip:
                    last_flip = time
                    flips.append(crank)
                    direction = -direction
                    cycles += direction > 0
                    goal = heads[direction]
                head_ready = abs(data.qpos[pitch_q] - goal) < math.radians(4)
                # ヨーサーボの指令が正のとき、クランク角は減る
                power = case.get("command_out", abs(case["command"])) if direction > 0 else case.get("command_back", abs(case["command"]))
                data.ctrl[0] = -direction * power * ramp if head_ready else 0.0
                if timed and not timed_started:
                    data.ctrl[0] = -math.copysign(0.3, (center - half) - crank) if time >= SETTLE + 0.5 else 0.0
        else:
            wave = math.sin(crank - phase)
            if case["wave"] == "square":
                wave = math.copysign(1.0, wave)
            goal = mid + amp * wave * ramp
            data.ctrl[0] = max(-1.0, min(1.0, case["command"] * ramp * (1 + case["speed_mod"] * math.sin(crank - speed_phase))))
        target += max(-sm.HEAD_PITCH_RATE * dt, min(sm.HEAD_PITCH_RATE * dt, goal - target))
        data.ctrl[1] = target
        position = data.qpos[:2].copy()
        mujoco.mj_step(model, data)
        heading = _heading(data.qpos[3:7])
        turn = math.remainder(heading - heading_prev, 2 * math.pi)
        heading_prev = heading
        if time >= SETTLE + 1.0:
            if start is None:
                start = (data.qpos[:2].copy(), data.qpos[shaft_q], time)
            heading_sum += turn
            path_len += float(np.linalg.norm(data.qpos[:2] - position))
            pitch_error = max(pitch_error, abs(data.qpos[pitch_q] - target))
            servo_peak = max(servo_peak, abs(data.actuator_force[0]))
            # 電流の見積もり: ストール電流 0.8 A × |デューティ − 速度 / 無負荷速度|(逆転の直後は 1 を超える)
            current = 0.8 * abs(data.ctrl[0] - data.qvel[table_v] / noload)
            current_sum += current
            current_sq_sum += current * current
            samples += 1
            if abs(data.actuator_force[0]) > 0.8 * stall:
                overload_steps += 1
                overload_run += 1
                overload_longest = max(overload_longest, overload_run)
            else:
                overload_run = 0
            if has_backlash:
                shaft_peak = max(shaft_peak, abs(data.sensordata[0]))
        if step % 100 == 0:
            tilt_pitch, tilt_roll = sm._tilt(data.qpos[3:7])
            hit = any(floor_id in (c.geom1, c.geom2) and ({c.geom1, c.geom2} & body_ids) for c in data.contact[: data.ncon])
            fallen = fallen or hit or abs(tilt_pitch) > 45 or abs(tilt_roll) > 45
        if renderer is not None and step % int(round((1 / 30) / dt)) == 0:
            renderer.update_scene(data, camera="track")
            frames.append(renderer.render().copy())
    if renderer is not None:
        import imageio

        imageio.mimsave(case["video"], frames, fps=30, quality=8, macro_block_size=None)
        renderer.close()
    revolutions = (data.qpos[shaft_q] - start[1]) / (2 * math.pi)
    duration = case["seconds"] - start[2]
    moved = data.qpos[:2] - start[0]
    turn_deg = math.degrees(heading_sum)
    return dict(
        case,
        cycles=cycles if rock else None,
        servo_peak_mNm=float(servo_peak * 1000),
        current_mean_A=float(current_sum / samples),
        current_rms_A=float(math.sqrt(current_sq_sum / samples)),
        overload_share=float(overload_steps / samples),
        overload_longest_s=float(overload_longest * dt),
        flips_deg=[float(math.degrees(v)) for v in flips],
        shaft_peak_mNm=float(shaft_peak * 1000) if has_backlash else None,
        end_tilt_deg=[float(v) for v in sm._tilt(data.qpos[3:7])],
        fallen=bool(fallen),
        crank_rpm=float(revolutions / duration * 60),
        turn_deg=float(turn_deg),
        turn_deg_per_rev=float(turn_deg / abs(revolutions)) if abs(revolutions) > 0.2 else 0.0,
        turn_deg_per_s=float(turn_deg / duration),
        moved_mm=float(np.linalg.norm(moved) * 1000),
        path_mm=float(path_len * 1000),
        head_lag_deg=float(math.degrees(pitch_error)),
    )


BASE = dict(friction=0.5, mass=0.110, cg_y=10.0, cg_z=0.0, command=0.35, mid=45.0, amp=45.0, phase=0.0, wave="square", speed_mod=0.0, speed_phase=0.0)


def cases(which: str, seconds: float) -> list[dict]:
    base = dict(BASE, seconds=seconds)
    out = []
    if which == "phase":
        # 頭を振らない場合(基準)と、頭を振る位相を 45° ずつ変えた場合。前進と後退
        for command in (0.35, -0.35):
            out.append(dict(base, command=command, amp=0.0))
            for phase in range(0, 360, 45):
                out.append(dict(base, command=command, phase=float(phase)))
    elif which == "sense":
        # 仮定を振る: 頭の重心の位置、動く質量、床の摩擦、振り方、歩く速さ
        for cg_y, mass, friction, wave, command in itertools.product((0.0, 10.0, 20.0), (0.08, 0.14), (0.3, 0.8), ("square", "sine"), (0.2, 0.55)):
            for phase in (BEST_PHASE, BEST_PHASE + 180.0):
                out.append(dict(base, cg_y=cg_y, mass=mass, friction=friction, wave=wave, command=command, phase=phase))
    elif which == "rock":
        # 往復: 往復の中心の位相を 30° ずつ、振れ幅 2 通り、頭の向きの組み合わせ 2 通り + 頭を変えない基準
        for half in (45.0, 90.0):
            for center in range(0, 360, 30):
                for head_out, head_back in ((0.0, 90.0), (90.0, 0.0), (0.0, 0.0)):
                    out.append(dict(base, wave="rock", rock_center=float(center), rock_half=half, head_out=head_out, head_back=head_back))
    elif which == "kick":
        # 往復の行きと帰りで回す強さを変える(強く回すと、ヨーサーボの反動で車体が床の上で回る向きに押される)
        for half in (30.0, 60.0):
            for center in range(0, 360, 30):
                for command_out, command_back in ((1.0, 0.15), (0.15, 1.0)):
                    for head in (0.0, 90.0):
                        out.append(dict(base, wave="rock", rock_center=float(center), rock_half=half, head_out=head, head_back=head, command_out=command_out, command_back=command_back))
    elif which == "speed":
        # 歩く速さも位相に合わせて変える
        for speed_mod in (0.5, 0.9):
            for speed_phase in range(0, 360, 45):
                out.append(dict(base, phase=BEST_PHASE, speed_mod=speed_mod, speed_phase=float(speed_phase)))
                out.append(dict(base, amp=0.0, speed_mod=speed_mod, speed_phase=float(speed_phase)))
    return out


BEST_PHASE = float(os.environ.get("STRIDER_TURN_PHASE", "0"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=14.0)
    parser.add_argument("--set", default="phase", choices=("phase", "sense", "speed", "rock", "kick"))
    parser.add_argument("--video", type=int)
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    args = parser.parse_args()
    sm.OUT.mkdir(parents=True, exist_ok=True)
    sm.build_xml(0.5, head=dict(mass=BASE["mass"], cg_y=0.0, cg_z=0.0))  # メッシュを書き出す
    todo = cases(args.set, args.seconds)
    if args.video is not None:
        todo = [dict(todo[args.video], video=str(sm.OUT / f"turn_{args.set}_{args.video}.mp4"))]
        rows = [simulate(todo[0])]
    else:
        with Pool(args.workers) as pool:
            rows = pool.map(simulate, todo, chunksize=1)
    print("番号 摩擦 質量g 重心前mm 指令  振り方 中心 振幅 位相 | 速さ変調 位相 | 転倒 rpm  旋回°/周  旋回°/秒  移動mm 頭の遅れ°")
    for index, row in enumerate(rows):
        print(
            f"{index:3d} {row['friction']:4.1f} {row['mass'] * 1000:5.0f} {row['cg_y']:7.1f} {row['command']:+5.2f}  {row['wave']:6s} {row['mid']:4.0f} {row['amp']:4.0f} {row['phase']:4.0f} | "
            f"{row['speed_mod']:6.1f} {row['speed_phase']:5.0f} | {'あり' if row['fallen'] else 'なし'} {row['crank_rpm']:+6.1f} {row['turn_deg_per_rev']:+8.2f} {row['turn_deg_per_s']:+8.2f} {row['moved_mm']:7.1f} {row['head_lag_deg']:6.1f}"
        )
    if args.set in ("rock", "kick"):
        print("往復の動かし方: 番号 中心 振れ幅 行きの頭 帰りの頭 行きの強さ 帰りの強さ | 転倒 往復回数 旋回°/往復 旋回°/秒 元の場所からのずれmm")
        for index, row in enumerate(rows):
            per_cycle = row["turn_deg"] / row["cycles"] if row["cycles"] else float("nan")
            print(f"{index:3d} {row['rock_center']:4.0f} {row['rock_half']:4.0f} {row['head_out']:4.0f} {row['head_back']:4.0f} {row.get('command_out', abs(row['command'])):5.2f} {row.get('command_back', abs(row['command'])):5.2f} | {'あり' if row['fallen'] else 'なし'} {row['cycles']:3d} {per_cycle:+7.2f} {row['turn_deg_per_s']:+7.2f} {row['moved_mm']:7.1f}")
    with open(sm.OUT / f"turn_{args.set}.json", "w", encoding="utf-8") as handle:
        json.dump(rows, handle, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())

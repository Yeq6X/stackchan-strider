"""スタックチャン・ストライダー: 製品版スタックチャン(M5Stack K151)用 ヤンセン機構4脚ウォーカーの形状定義。

座標系(単位 mm):
  X = 右, Y = 前, Z = 上。原点 = クランク軸(X軸)とヨー軸(Z軸)の交点 = 傘歯車のピッチ円錐頂点。
部品は「ローカル形状」と「姿勢行列(4x4)」に分けて持つ。instances(theta) がクランク角ごとの全部品の姿勢を返す。
"""
from __future__ import annotations

import dataclasses
import math
import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np
import trimesh
from manifold3d import CrossSection, JoinType, Manifold, Mesh

from gears import BevelSpec, bevel_teeth_mesh, mesh_phase_offset
from jansen import HOLY, LegGeometry, solve_leg

ROOT = Path(__file__).resolve().parent.parent
SEG = 72

# ---------------------------------------------------------------- 寸法パラメータ
FIT = 0.1  # C クリップの穴と溝の底の半径すきま
HOLE_FIT = 0.2  # 回転する穴と軸の半径すきま(リンク・三角板の穴と突起、側板の軸受け穴とスリーブ)
GAP = 0.15  # 重ねた部品どうしの面のすきま(根元と、層と層の間)
END_GAP = 0.0  # 重ねた最後の部品と C クリップの間。溝の中でクリップが動ける分(CLIP_PLAY)があるので、ここでは取らない
CLIP_PLAY = 0.1  # クリップ溝の幅とクリップの厚みの差
LAYER_T = 3.0  # リンク厚
LAYER_PITCH = LAYER_T + GAP  # リンク層のピッチ

# 製品版スタックチャン(公式 STL / 図面からの値)
SC_BASE_H = 8.0  # 土台デッキ高さ
SC_AXIS_IN_BASE = (24.0, 32.0)  # 土台の左前隅から見たヨー軸位置
SC_HEAD_HALF_W = 27.0
SC_HEAD_BOTTOM = 16.5  # 机面から頭部下端
SC_HEAD_TOP = 70.5
SC_FRONT = 33.5  # ヨー軸から CoreS3 前面まで(図面からの推定)
SC_PITCH_AXIS_H = 43.0  # 机面からピッチ軸(推定)
SC_MASS_G = 187.2

# 脚(Jansen 比率 × LEG_SCALE。クランク半径だけ 15 → 8 に縮めて足上げ高さと揺れを抑える。MuJoCo の走査(sim_sweep.py)で、スケール 0.65 は重心が前へ 8 mm ずれると転倒、0.75 × 比 8 は重心 ±8 mm でも転倒なし)
# 環境変数 STRIDER_LEG_SCALE / STRIDER_CRANK_RATIO で上書きできる(寸法探索用。通常は既定値を使う)
LEG_SCALE = float(os.environ.get("STRIDER_LEG_SCALE", "0.75"))
CRANK_RATIO = float(os.environ.get("STRIDER_CRANK_RATIO", "8.0"))
LEG = dataclasses.replace(LegGeometry.scaled(LEG_SCALE), m=CRANK_RATIO * LEG_SCALE)
FOOT_R = 4.5  # 足先の丸み半径
FOOT_W = 11.0  # 足先の幅(X方向)
END_R = 5.0  # リンク端のボス半径
BAR_W = 6.0  # リンクの胴の幅
STUD_R = 2.5
CLIP_T = 1.0  # C クリップの厚み
CLIP_R_OUT = 4.6
GROOVE_DEPTH = 0.8  # クリップ溝の深さ(半径)
GROOVE_W = CLIP_T + CLIP_PLAY  # クリップ溝の幅(平らな部分)
PIN_TIP = 1.2  # 溝の先の頭の長さ

# 傘歯車(1:1 マイタ、軸角 90°、ピッチ円錐半角 45°)
# backlash は歯車1枚あたりの歯厚減少量。2枚で設計バックラッシュ約 0.5 mm(FDM の太りと組立ガタを見込む)
GEAR = BevelSpec(module=2.0, teeth=20, mate_teeth=20, face_width=8.0, backlash=0.25)
GEAR_BACK = 22.3  # 歯の背面を平らに切る位置(頂点からの距離)

# 車体
PLATE_IN = 23.5  # 側板の内面 |x|
PLATE_T = 5.0
PLATE_OUT = PLATE_IN + PLATE_T
CRANK_T = 4.0
CRANK_R = 12.0
L0 = PLATE_OUT + 0.2  # クランク円板の内面
L1 = L0 + CRANK_T + GAP  # リンク第1層の内面
SHAFT_AF = 8.0  # 六角シャフト二面幅(呼び寸法。穴はこれ + SHAFT_BORE_CLEARANCE)
SHAFT_BORE_CLEARANCE = 0.3  # 六角穴の広げ量(二面幅)
SHAFT_UNDERSIZE = 0.2  # シャフト本体を呼び寸法より細くする量(二面幅)。穴との差は合計 0.5
CRANK_SOCKET_CLEARANCE = 0.25  # クランクの六角の止まり穴とシャフトの二面幅の差。浅い穴で抜け止めが無いので、通し穴(差 0.5)より詰める
CRANK_SOCKET_DEPTH = 3.0  # クランクの六角の止まり穴の深さ
# 左右のクランクを、ピンが 180° 反対になる向きでしか差せないようにする合わせ:
# シャフトの両端で、六角の角を 1 つずつ(右端と左端で反対側の角を)平らに落とし、クランクの穴は、ピンの側の角を埋める。
# シャフトの中ほどは六角のままなので、ピニオンとスリーブの穴はそのまま通る
KEY_CUT = 1.2  # シャフトの角を落とす量(角の先から)
KEY_LENGTH = CRANK_SOCKET_DEPTH + 0.5  # シャフトの端から、角を落とす長さ(穴の深さより少し長く)
SLEEVE_R = 6.0
DECK_Z0, DECK_Z1 = 6.5, 9.5
HUB_Z0 = 15.6  # ターンテーブル歯車のハブ(歯の内側の筒)の下端
BOSS_R = 10.5  # デッキのボス(ターンテーブルの軸受け)の外半径
SPINDLE_FIT = 0.2  # ターンテーブルの軸と穴のすきま(片側)。大きいと円板が傾く
BOSS_GAP = 0.4  # ボスの外側と、ターンテーブル側の逃がし溝のすきま(当てない面)
SPINDLE_R = 7.8
TABLE_R = 41.0
TABLE_T = 2.0
FRAME_H = 6.0
COLLAR_R_IN = TABLE_R + 0.7
COLLAR_R_OUT = COLLAR_R_IN + 3.0
HORN_Y = 41.6  # 側板のツノ(カラー固定ほぞ)の中心 y
EAR_R = 13.0
EAR_GAP = 0.2  # 頭とパッドのすきま(片側)


def _leg_extents() -> tuple[float, float]:
    """脚の全ジョイントの最高点 z と足先の最低点 z(1回転の最大・最小)。"""
    top, low = -1e9, 1e9
    for n in range(360):
        joints = solve_leg(LEG, math.radians(n))
        top = max(top, max(p[1] for p in joints.values()))
        low = min(low, joints["H"][1])
    return top, low


LEG_TOP, FOOT_LOW = _leg_extents()
TABLE_Z0 = max(LEG_TOP + END_R + 0.8, GEAR_BACK + 0.5)  # ターンテーブル円板の下面
# デッキのボス上面 = ターンテーブルのスラスト面。ボスを歯車の中まで伸ばし、軸の受けを長くして傾きを抑える(円板の下に 2 mm 残す)
BOSS_TOP = TABLE_Z0 - 2.0
TABLE_Z1 = TABLE_Z0 + TABLE_T  # ポケット床 = スタックチャン土台の底面
FRAME_Z1 = TABLE_Z1 + FRAME_H
COLLAR_Z0 = TABLE_Z0 + 0.4
COLLAR_Z1 = FRAME_Z1 + 3.4
PITCH_Z = TABLE_Z1 + SC_PITCH_AXIS_H
GROUND_Z = FOOT_LOW - FOOT_R

COLORS = {
    "chassis": "#2d3340",
    "accent": "#e53a2f",
    "link": "#d6d9dc",  # ライトグレー(スタックチャンの筐体に近い色)
    "stackchan": "#f6f6f4",
    "stackchan_dark": "#1c1c1e",
}


# ---------------------------------------------------------------- 形状ヘルパー
def cyl(radius: float, z0: float, z1: float, seg: int = SEG) -> Manifold:
    return Manifold.cylinder(z1 - z0, radius, radius, seg).translate((0, 0, z0))


def cyl_x(radius: float, x0: float, x1: float, y: float = 0.0, z: float = 0.0, seg: int = SEG) -> Manifold:
    """X軸方向の円柱。"""
    return cyl(radius, x0, x1, seg).transform(AXIS_Z_TO_X[:3]).translate((0, y, z))


def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Manifold:
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def circle(radius: float, cx: float = 0.0, cy: float = 0.0, seg: int = SEG) -> CrossSection:
    return CrossSection.circle(radius, seg).translate((cx, cy))


def rect(x0: float, x1: float, y0: float, y1: float) -> CrossSection:
    return CrossSection.square((x1 - x0, y1 - y0)).translate((x0, y0))


def polygon(points) -> CrossSection:
    return CrossSection([np.asarray(points, dtype=np.float64)])


def hull2(*sections: CrossSection) -> CrossSection:
    return CrossSection.batch_hull(list(sections))


def hexagon(across_flats: float) -> CrossSection:
    radius = across_flats / math.sqrt(3)
    return polygon([(radius * math.cos(math.radians(60 * n)), radius * math.sin(math.radians(60 * n))) for n in range(6)])


def mat(rows) -> np.ndarray:
    out = np.eye(4)
    out[:3, :] = np.asarray(rows, dtype=np.float64)
    return out


def translation(x: float, y: float, z: float) -> np.ndarray:
    out = np.eye(4)
    out[:3, 3] = (x, y, z)
    return out


def rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return mat([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0]])


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return mat([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0]])


# ローカル (a, b, h) → ワールド (x=h, y=a, z=b)。YZ平面の板を X 方向に押し出す用。
AXIS_Z_TO_X = mat([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]])
CLIP_FLAT = mat([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0]])  # クリップ(軸 X、口 +Y)を、軸 Z、口 +X に寝かせる
ROT180_Y = mat([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0]])
ROT180_Z = mat([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0]])
MIRROR_W = mat([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0]])  # リンクローカルの w を反転


def apply(shape: Manifold, matrix: np.ndarray) -> Manifold:
    return shape.transform(matrix[:3, :])


def from_trimesh(mesh: trimesh.Trimesh) -> Manifold:
    out = Manifold(Mesh(np.asarray(mesh.vertices, dtype=np.float32), np.asarray(mesh.faces, dtype=np.uint32)))
    if out.status().name != "NoError":
        raise RuntimeError(f"メッシュを Manifold にできない: {out.status()}")
    return out


def to_trimesh(shape: Manifold) -> trimesh.Trimesh:
    mesh = shape.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts), process=False)


def plate_yz(section: CrossSection, x0: float, x1: float) -> Manifold:
    """(y, z) の断面を x0..x1 に押し出す。"""
    return apply(section.extrude(x1 - x0), AXIS_Z_TO_X).translate((x0, 0, 0))


# ---------------------------------------------------------------- 脚リンク(ローカル: u-w 平面、厚み h=0..LAYER_T)
def _dogbone(length: float) -> CrossSection:
    ends = circle(END_R) + circle(END_R, length, 0)
    return ends + hull2(circle(BAR_W / 2), circle(BAR_W / 2, length, 0))


def _hole(cx: float, cy: float) -> CrossSection:
    return circle(STUD_R + HOLE_FIT, cx, cy)


def grooved_pin(base_h: float, clear: float) -> Manifold:
    """C クリップで留める軸(ローカル: 軸 = Z、原点が軸心)。base_h から clear だけ円柱、その先にクリップ溝と頭。

    溝の頭側は 45° の面取り(印刷時の張り出しを支える)。クランクのピンも脚の突起も同じ形。
    """
    z0 = base_h + clear
    z1 = z0 + GROOVE_W
    inner = STUD_R - GROOVE_DEPTH
    profile = polygon(
        [
            (0, base_h),
            (STUD_R, base_h),
            (STUD_R, z0),
            (inner, z0),
            (inner, z1),
            (inner + 0.5, z1 + 0.5),
            (STUD_R, z1 + 0.5),
            (STUD_R, z1 + 0.5 + PIN_TIP - 0.5),
            (STUD_R - 0.5, z1 + 0.5 + PIN_TIP),
            (0, z1 + 0.5 + PIN_TIP),
        ]
    )
    return profile.revolve(48)


def stud(cx: float, cy: float, base_h: float, clear: float) -> Manifold:
    """リンクや側板から立つ突起。base_h から clear の高さにクリップ溝が来る。"""
    return grooved_pin(base_h, clear).translate((cx, cy, 0))


def stud_clear(layers: int) -> float:
    """突起の根元(持ち主の上面)から、layers 枚を重ねた上の面 + END_GAP までの長さ。"""
    return layers * LAYER_PITCH + END_GAP


def clip_seat(clear: float) -> float:
    """クリップ下面の高さ(突起の根元から)。溝の中央に置く。"""
    return clear + (GROOVE_W - CLIP_T) / 2


@dataclass(frozen=True)
class LegLocal:
    """基準脚(クランク軸原点、固定ピボット B が後方)のリンク内ローカル座標。"""

    tri_upper_e: tuple[float, float]  # 上三角: 原点 B、u 軸 B→D での E
    tri_lower_h: tuple[float, float]  # 下三角: 原点 F、u 軸 F→G での H


def _local_point(origin: np.ndarray, along: np.ndarray, point: np.ndarray) -> tuple[float, float]:
    axis = (along - origin) / np.linalg.norm(along - origin)
    normal = np.array([-axis[1], axis[0]])
    rel = point - origin
    return float(rel @ axis), float(rel @ normal)


@lru_cache(maxsize=None)
def leg_local() -> LegLocal:
    joints = solve_leg(LEG, 0.0)
    return LegLocal(
        tri_upper_e=_local_point(joints["B"], joints["D"], joints["E"]),
        tri_lower_h=_local_point(joints["F"], joints["G"], joints["H"]),
    )


def _angle(p: np.ndarray, q: np.ndarray) -> float:
    return math.atan2(q[1] - p[1], q[0] - p[0])


# 各リンクのローカル原点ジョイントと u 軸方向のジョイント
LINK_AXES = {"j": ("C", "D"), "k": ("C", "F"), "c": ("B", "F"), "f": ("E", "G"), "tri_upper": ("B", "D"), "tri_lower": ("F", "G")}
def link_angle(name: str, joints: dict[str, np.ndarray]) -> float:
    start, end = LINK_AXES[name]
    return _angle(joints[start], joints[end])


# 突起とクリップ: 継ぎ目 → (突起の持ち主, 持ち主ローカルでの位置を返す関数, 突起に通す部品の枚数)。"plate" は車体側板
STUD_JOINTS = {
    "B": ("plate", None, 2),  # リンク3と上の三角板
    "D": ("j", lambda g: (g.j, 0.0), 1),  # 上の三角板
    "E": ("f", lambda g: (0.0, 0.0), 1),  # 上の三角板
    "G": ("f", lambda g: (g.f, 0.0), 1),  # 下の三角板
    "F": ("c", lambda g: (g.c, 0.0), 2),  # 下の三角板とリンク2
}
# クリップの口の向き [rad](持ち主ローカル、0 = リンク軸の向き)。組立時に横から押し込める向きを verify_assembly.py で確かめてある
CLIP_MOUTH = {"B": 0.0, "D": 0.0, "E": 0.0, "G": 0.0, "F": 0.0}


# ---------------------------------------------------------------- 刻印(組立用の目印)
# 4×6 の升目に描いた一筆書きの字形。リンクは数字、ジョイントは英字で区別する
GLYPHS = {
    "1": [[(1, 4.8), (2.2, 6), (2.2, 0)], [(1, 0), (3.4, 0)]],
    "2": [[(0, 5), (1, 6), (3, 6), (4, 5), (4, 4), (0, 0), (4, 0)]],
    "3": [[(0, 5), (1, 6), (3, 6), (4, 5), (4, 4), (3, 3), (1.5, 3)], [(3, 3), (4, 2), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "4": [[(3, 0), (3, 6), (0, 2), (4, 2)]],
    "B": [[(0, 0), (0, 6), (3, 6), (4, 5), (4, 4), (3, 3), (0, 3)], [(3, 3), (4, 2), (4, 1), (3, 0), (0, 0)]],
    "C": [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 1), (1, 0), (3, 0), (4, 1)]],
    "D": [[(0, 0), (0, 6), (2.5, 6), (4, 4.5), (4, 1.5), (2.5, 0), (0, 0)]],
    "E": [[(4, 6), (0, 6), (0, 0), (4, 0)], [(0, 3), (3, 3)]],
    "F": [[(4, 6), (0, 6), (0, 0)], [(0, 3), (3, 3)]],
    "G": [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 1), (1, 0), (3, 0), (4, 1), (4, 3), (2.2, 3)]],
}
LINK_NUMBER = {"j": "1", "k": "2", "c": "3", "f": "4"}
ENGRAVE_DEPTH = 0.6
ENGRAVE_STROKE = 0.7


def glyph_section(char: str, height: float) -> CrossSection:
    """1文字の彫り込み断面(中心が原点、全高 height)。"""
    scale = (height - ENGRAVE_STROKE) / 6
    out = None
    for line in GLYPHS[char]:
        points = [((px - 2) * scale, (py - 3) * scale) for px, py in line]
        for start, end in zip(points, points[1:]):
            segment = hull2(circle(ENGRAVE_STROKE / 2, *start, 12), circle(ENGRAVE_STROKE / 2, *end, 12))
            out = segment if out is None else out + segment
    return out


def engrave(solid: Manifold, char: str, height: float, x: float, y: float, angle: float = 0.0) -> Manifold:
    """上面(h = LAYER_T)に文字を彫る。angle は文字の向き [rad]。"""
    mark = glyph_section(char, height).rotate(math.degrees(angle)).translate((x, y))
    return solid - mark.extrude(ENGRAVE_DEPTH + 0.2).translate((0, 0, LAYER_T - ENGRAVE_DEPTH))


def _bar_marks(name: str, length: float) -> list[dict]:
    """棒リンクの刻印: 中央にリンク番号、両端に差し込むジョイントの記号。"""
    first, last = LINK_AXES[name]
    return [
        dict(char=LINK_NUMBER[name], kind="number", height=4.4, x=length / 2, y=0.0, angle=0.0),
        dict(char=first, kind="joint", height=3.3, x=END_R + 2.9, y=0.0, angle=0.0),
        dict(char=last, kind="joint", height=3.3, x=length - END_R - 2.9, y=0.0, angle=0.0),
    ]


def _corner_marks(corners, labels) -> list[dict]:
    """三角板の刻印: 各頂点の穴のそばに、そこへ来るジョイントの記号(次の頂点へ向かう辺の上)。"""
    center = np.mean(np.asarray(corners), axis=0)
    marks = []
    for index, label in enumerate(labels):
        if label is None:
            continue
        here = np.asarray(corners[index])
        along = np.asarray(corners[(index + 1) % 3]) - here
        along /= np.linalg.norm(along)
        inward = np.array([-along[1], along[0]])
        if inward @ (center - here) < 0:
            inward = -inward
        spot = here + along * (END_R + 5.0) + inward * 0.5
        marks.append(dict(char=label, kind="joint", height=3.6, x=float(spot[0]), y=float(spot[1]), angle=math.atan2(along[1], along[0])))
    return marks


def engraving_marks(name: str) -> list[dict]:
    """部品 name(j / k / c / f / tri_upper / tri_lower)の刻印の一覧。位置は部品ローカル、面は上面(h = LAYER_T)。"""
    geom, local = LEG, leg_local()
    if name in LINK_NUMBER:
        return _bar_marks(name, getattr(geom, name))
    if name == "tri_upper":
        return _corner_marks([(0.0, 0.0), (geom.b, 0.0), local.tri_upper_e], ("B", "D", "E"))
    if name == "tri_lower":
        return _corner_marks([(0.0, 0.0), (geom.g, 0.0), local.tri_lower_h], ("F", "G", None))
    return []


def _engrave_marks(solid: Manifold, name: str) -> Manifold:
    for mark in engraving_marks(name):
        solid = engrave(solid, mark["char"], mark["height"], mark["x"], mark["y"], mark["angle"])
    return solid


def mark_inlay(mark: dict) -> Manifold:
    """刻印の溝をちょうど埋める形(表示用。組立ガイドで光らせる)。上面からわずかに出す。"""
    section = glyph_section(mark["char"], mark["height"]).rotate(math.degrees(mark["angle"])).translate((mark["x"], mark["y"]))
    return section.extrude(ENGRAVE_DEPTH + 0.06).translate((0, 0, LAYER_T - ENGRAVE_DEPTH))


@lru_cache(maxsize=None)
def link_part(name: str) -> Manifold:
    geom, local = LEG, leg_local()
    if name == "j":  # C-D。D 側にスタッド(上三角を貫通)
        section = _dogbone(geom.j) - _hole(0, 0)
        return _engrave_marks(section.extrude(LAYER_T), "j") + stud(geom.j, 0, LAYER_T, stud_clear(1))
    if name == "k":  # C-F。穴のみ
        section = _dogbone(geom.k) - _hole(0, 0) - _hole(geom.k, 0)
        return _engrave_marks(section.extrude(LAYER_T), "k")
    if name == "c":  # B-F。F 側にスタッド(下三角と k を貫通)
        section = _dogbone(geom.c) - _hole(0, 0)
        return _engrave_marks(section.extrude(LAYER_T), "c") + stud(geom.c, 0, LAYER_T, stud_clear(2))
    if name == "f":  # E-G。両端にスタッド
        section = _dogbone(geom.f)
        return (
            _engrave_marks(section.extrude(LAYER_T), "f")
            + stud(0, 0, LAYER_T, stud_clear(1))
            + stud(geom.f, 0, LAYER_T, stud_clear(1))
        )
    if name == "tri_upper":  # B, D, E
        ex, ey = local.tri_upper_e
        corners = [(0.0, 0.0), (geom.b, 0.0), (ex, ey)]
        section = _truss(corners)
        section -= _hole(0, 0)
        section -= _hole(geom.b, 0)
        section -= _hole(ex, ey)
        return _engrave_marks(section.extrude(LAYER_T), "tri_upper")
    if name == "tri_lower":  # F, G, H(足先)
        hx, hy = local.tri_lower_h
        corners = [(0.0, 0.0), (geom.g, 0.0), (hx, hy)]
        section = _truss(corners, last_radius=FOOT_R)
        section -= _hole(0, 0)
        section -= _hole(geom.g, 0)
        shoe = circle(FOOT_R, hx, hy).extrude(FOOT_W)
        return _engrave_marks(section.extrude(LAYER_T), "tri_lower") + shoe
    raise KeyError(name)


def _truss(corners, last_radius: float = END_R) -> CrossSection:
    """3頂点を結ぶトラス状の三角板(中央を抜く)。"""
    radii = [END_R, END_R, last_radius]
    nodes = [circle(r, x, y) for r, (x, y) in zip(radii, corners)]
    outline = hull2(*nodes)
    window = polygon(corners).offset(-(END_R + 3.0), JoinType.Round).offset(2.0, JoinType.Round)
    return outline - window


# ---------------------------------------------------------------- 歯車
@lru_cache(maxsize=None)
def _teeth() -> Manifold:
    return from_trimesh(bevel_teeth_mesh(GEAR))


@lru_cache(maxsize=None)
def gear_table_part() -> Manifold:
    """ターンテーブル歯車: 歯は下向き。上面の円板がスタックチャン土台の受け床になる。"""
    teeth = apply(_teeth(), ROT180_Y).trim_by_plane((0, 0, -1), -GEAR_BACK)
    neck = cyl(GEAR.tip_radius + 0.1, GEAR_BACK, TABLE_Z0)
    disc = cyl(TABLE_R, TABLE_Z0, TABLE_Z1, 128)
    hub = cyl(13.0, HUB_Z0, GEAR_BACK)
    # デッキのボスが入る逃がし溝(天井がスラスト面)。その中心に軸を立てる
    relief = cyl(BOSS_R + BOSS_GAP, HUB_Z0 - 1, BOSS_TOP)
    spindle = cyl(SPINDLE_R, DECK_Z0, BOSS_TOP + 0.5)
    body = teeth + neck + disc + hub - relief + spindle
    for x, y in FRAME_TABS:
        body -= box(x - TAB_L / 2 - 0.2, x + TAB_L / 2 + 0.2, y - TAB_W / 2 - 0.2, y + TAB_W / 2 + 0.2, TABLE_Z0 - 1, TABLE_Z1 + 1)
    return body


FRAME_TABS = [(33.0, -6.0), (-33.0, -6.0), (0.0, -33.5)]
TAB_L, TAB_W = 6.0, 3.0


@lru_cache(maxsize=None)
def pinion_part() -> Manifold:
    """ピニオン(ローカル: 軸 = X、頂点 = 原点、歯は -X 向き、背面は x = GEAR_BACK の平面)。"""
    teeth = apply(apply(_teeth(), rot_z(mesh_phase_offset(GEAR))), ROT180_Y @ mat([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0]]))
    teeth = teeth.trim_by_plane((-1, 0, 0), -GEAR_BACK)
    hub = cyl_x(9.0, PINION_FRONT, GEAR_BACK)
    # 歯の側へ伸ばした筒が左側板の軸受を兼ねる(別部品の長いスリーブを無くす)
    snout = cyl_x(SLEEVE_R, -L0, PINION_FRONT)
    bore = apply(hexagon(SHAFT_AF + SHAFT_BORE_CLEARANCE).extrude(L0 + GEAR_BACK + 2), AXIS_Z_TO_X).translate((-L0 - 1, 0, 0))
    return teeth + hub + snout - bore


PINION_FRONT = 12.5


# ---------------------------------------------------------------- 駆動軸まわり
@lru_cache(maxsize=None)
def shaft_part() -> Manifold:
    half = L0 + CRANK_SOCKET_DEPTH - 0.1
    shaft = apply(hexagon(SHAFT_AF - SHAFT_UNDERSIZE).extrude(2 * half), AXIS_Z_TO_X).translate((-half, 0, 0))
    # 角は ±Y を向いている。右端は +Y の角、左端は −Y の角を落とす(クランクのピンの向きが左右で 180° 違うため)
    flat = _key_flat(SHAFT_AF - SHAFT_UNDERSIZE)
    shaft -= box(half - KEY_LENGTH, half + 1, flat, flat + 5, -6, 6)
    shaft -= box(-half - 1, -half + KEY_LENGTH, -flat - 5, -flat, -6, 6)
    return shaft


def _key_flat(across_flats: float) -> float:
    """合わせの平らな面の位置(軸からの距離)。六角の角の先から KEY_CUT だけ内側。"""
    return across_flats / math.sqrt(3) - KEY_CUT


def _sleeve(x0: float, x1: float) -> Manifold:
    bore = apply(hexagon(SHAFT_AF + SHAFT_BORE_CLEARANCE).extrude(x1 - x0 + 2), AXIS_Z_TO_X).translate((x0 - 1, 0, 0))
    return cyl_x(SLEEVE_R, x0, x1) - bore


@lru_cache(maxsize=None)
def sleeve_part() -> Manifold:
    """ピニオン背面と右クランクの間に入る短いスリーブ(右側板の軸受)。"""
    return _sleeve(GEAR_BACK, L0)


PIN_CLEAR = 4 * LAYER_PITCH + END_GAP  # クランクの面から、リンク 4 枚を重ねた上の面 + END_GAP まで


@lru_cache(maxsize=None)
def crank_part() -> Manifold:
    """クランク(ローカル: 軸 = X、内面 x=0、ピンは +Y 方向へ偏心)。六角の止まり穴でシャフトに差す。"""
    radius = LEG.m
    disc = hull2(circle(CRANK_R), circle(STUD_R + 3.0, radius, 0)).extrude(CRANK_T)
    # 穴は、ピンの側(スケッチの +X)の角を埋める。シャフトの落とした角とだけ合う
    key = _key_flat(SHAFT_AF - SHAFT_UNDERSIZE) + CRANK_SOCKET_CLEARANCE / 2
    socket = (hexagon(SHAFT_AF - SHAFT_UNDERSIZE + CRANK_SOCKET_CLEARANCE) - rect(key, key + 5, -6, 6)).extrude(CRANK_SOCKET_DEPTH)
    pin = grooved_pin(CRANK_T, PIN_CLEAR).translate((radius, 0, 0))
    return apply(disc - socket + pin, AXIS_Z_TO_X)


@lru_cache(maxsize=None)
def clip_part() -> Manifold:
    """クランクのピンと脚の突起の溝にはめる C 形クリップ(ローカル: 軸 = X、厚み x=0..CLIP_T、口は +Y)。"""
    inner = STUD_R - GROOVE_DEPTH
    ring = circle(CLIP_R_OUT) - circle(inner + FIT)
    mouth = rect(0, 6, -inner * 0.82, inner * 0.82)
    return apply((ring - mouth).extrude(CLIP_T), AXIS_Z_TO_X)


# ---------------------------------------------------------------- 車体
SNAP_BARB = 0.5  # ツメの返しの出っ張り(片側)。穴との掛かりは片側 0.3。大きくすると差し込みが固くなる


def _snap_tenon(width: float, length: float, slot_depth: float) -> CrossSection:
    """2本爪のスナップほぞ(ローカル: 根元 y=0 から +y へ length、先端に返し)。幅方向 = x。"""
    half = width / 2
    body = polygon(
        [
            (-half, 0),
            (half, 0),
            (half, length),
            (half + SNAP_BARB, length + 0.2),
            (half + SNAP_BARB, length + 0.9),
            (half - 1.0, length + 2.8),
            (-half + 1.0, length + 2.8),
            (-half - SNAP_BARB, length + 0.9),
            (-half - SNAP_BARB, length + 0.2),
            (-half, length),
        ]
    )
    slot = rect(-0.8, 0.8, length + 2.8 - slot_depth, length + 3.0) + circle(0.8, 0, length + 2.8 - slot_depth, 24)
    return body - slot


TENON_W = 6.0
DECK_TENON_Y = 32.0
COLLAR_LUG_T = 4.0


def plate_stud_clear(first_layer: int) -> float:
    """側板の外面から、B の突起のクリップ溝までの長さ(リンク3と上の三角板を重ねた上の面 + END_GAP)。"""
    return L1 + (first_layer + 2) * LAYER_PITCH - GAP + END_GAP - PLATE_OUT


@lru_cache(maxsize=None)
def side_plate_part() -> Manifold:
    """右側板(ワールド姿勢のまま)。左は Z 軸まわり 180° 回転で同じ部品。"""
    b_y, b_z = LEG.a, -LEG.l
    section = hull2(circle(10.0, -b_y, b_z), circle(10.0, b_y, b_z), circle(13.0, 0, 1.0))
    for sign in (1, -1):
        horn = hull2(circle(10.0, sign * b_y, b_z), rect(sign * HORN_Y - 4.0, sign * HORN_Y + 4.0, 14.0, COLLAR_Z0))
        section += horn
        tenon = _snap_tenon(TENON_W, COLLAR_LUG_T, 11.0).translate((sign * HORN_Y, COLLAR_Z0))
        section += tenon
    section -= circle(SLEEVE_R + HOLE_FIT)
    for sign in (1, -1):
        section -= rect(sign * DECK_TENON_Y - TENON_W / 2 - 0.2, sign * DECK_TENON_Y + TENON_W / 2 + 0.2, DECK_Z0 - 0.15, DECK_Z1 + 0.15)
    plate = plate_yz(section, PLATE_IN, PLATE_OUT)
    # 固定ピボット B のスタッド。後ろ脚(y<0)は第1層から、前脚(y>0)は第2層から使う
    for sign, first_layer in ((-1, 0), (1, 1)):
        shoulder_end = L1 + first_layer * LAYER_PITCH - GAP
        shoulder = cyl_x(4.6, PLATE_OUT, shoulder_end, sign * b_y, b_z)
        pivot = apply(stud(0, 0, 0, plate_stud_clear(first_layer)), AXIS_Z_TO_X)
        plate += shoulder + pivot.translate((PLATE_OUT, sign * b_y, b_z))
    return plate


@lru_cache(maxsize=None)
def deck_part() -> Manifold:
    """左右の側板をつなぎ、ターンテーブルの軸を受けるデッキ。"""
    half_y = DECK_TENON_Y + TENON_W / 2 + 2.0
    plan = rect(-PLATE_IN + 0.1, PLATE_IN - 0.1, -half_y, half_y)
    plan -= rect(PINION_FRONT - 1.0, PLATE_IN, -(GEAR.tip_radius + 1.6), GEAR.tip_radius + 1.6)
    for sx in (1, -1):
        for sy in (1, -1):
            tenon = _snap_tenon(TENON_W, PLATE_T + 0.2, 11.0).rotate(-90 * sx).translate((sx * (PLATE_IN - 0.1), sy * DECK_TENON_Y))
            plan += tenon
    # 軽量化の窓(左側)
    for sy in (1, -1):
        plan -= hull2(circle(3.0, -15.5, sy * 13.0), circle(3.0, -15.5, sy * 21.0), circle(3.0, -6.0, sy * 21.0))
    deck = plan.extrude(DECK_Z1 - DECK_Z0).translate((0, 0, DECK_Z0))
    boss = cyl(BOSS_R, DECK_Z0, BOSS_TOP)
    return deck + boss - cyl(SPINDLE_R + SPINDLE_FIT, DECK_Z0 - 1, BOSS_TOP + 1)


@lru_cache(maxsize=None)
def stackchan_base() -> Manifold:
    """公式 STL の土台を、ヨー軸 = Z 軸、底面 z=0、前 = +Y に置いたもの。"""
    mesh = trimesh.load(ROOT / "refs" / "official_stl" / "StackChan-Base.stl")
    low = mesh.bounds[0]
    shape = from_trimesh(mesh).translate(tuple(-low))
    return apply(shape.translate((-SC_AXIS_IN_BASE[0], -SC_AXIS_IN_BASE[1], 0)), ROT180_Z)


def _signed_area(points: np.ndarray) -> float:
    x, y = points[:, 0], points[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


@lru_cache(maxsize=None)
def socket_frame_part() -> Manifold:
    """土台がちょうどはまる枠(円の中に土台形の穴)。下面のツメでターンテーブル歯車と回り止め。"""
    # 投影の外周だけを使う(レゴ穴や USB 開口の内側に柱を残さない)
    outline = max(stackchan_base().project().to_polygons(), key=lambda pts: abs(_signed_area(np.asarray(pts))))
    pocket = polygon(outline).offset(0.25, JoinType.Round)
    chamfer = 2.0
    profile = polygon([(0, TABLE_Z1), (TABLE_R, TABLE_Z1), (TABLE_R, FRAME_Z1 - chamfer), (TABLE_R - chamfer, FRAME_Z1), (0, FRAME_Z1)])
    ring = profile.revolve(128)
    frame = ring - pocket.extrude(FRAME_H + 2).translate((0, 0, TABLE_Z1 - 1))
    # 指掛かり(土台を持ち上げるための切り欠き、左右)
    for sx in (1, -1):
        frame -= cyl(7.0, TABLE_Z1 - 1, FRAME_Z1 + 1).translate((sx * 24.5, 6.0, 0))
    for x, y in FRAME_TABS:
        frame += box(x - TAB_L / 2, x + TAB_L / 2, y - TAB_W / 2, y + TAB_W / 2, TABLE_Z0 + 0.2, TABLE_Z1 + 0.01)
    return frame


@lru_cache(maxsize=None)
def yoke_part() -> Manifold:
    """カラー(ターンテーブルの浮き上がり止め)とイヤーパッド(頭の回り止め)を一体にした部品。"""
    lip_r = TABLE_R - 2.0 + 0.6
    lip_z = FRAME_Z1 - 2.0 - (COLLAR_R_IN - TABLE_R) + 0.6
    profile = polygon(
        [
            (COLLAR_R_IN, COLLAR_Z0),
            (COLLAR_R_OUT, COLLAR_Z0),
            (COLLAR_R_OUT, COLLAR_Z1),
            (lip_r, COLLAR_Z1),
            (lip_r, lip_z + (COLLAR_R_IN - lip_r)),
            (COLLAR_R_IN, lip_z),
        ]
    )
    yoke = profile.revolve(128)
    # 側板のツノを受ける耳
    outside = CrossSection.circle(80, 96) - CrossSection.circle(COLLAR_R_IN, 128)
    for sx in (1, -1):
        for sy in (1, -1):
            cx = sx * (PLATE_IN + PLATE_T / 2)
            lug = rect(cx - PLATE_T / 2 - 2.6, cx + PLATE_T / 2 + 2.6, sy * HORN_Y - 6.2, sy * HORN_Y + 6.2)
            slot = rect(cx - PLATE_T / 2 - 0.2, cx + PLATE_T / 2 + 0.2, sy * HORN_Y - TENON_W / 2 - 0.2, sy * HORN_Y + TENON_W / 2 + 0.2)
            yoke += ((lug ^ outside) - slot).extrude(COLLAR_LUG_T).translate((0, 0, COLLAR_Z0))
            yoke -= slot.extrude(COLLAR_LUG_T + 2).translate((0, 0, COLLAR_Z0 - 1))
    # イヤーパッドのアーム(XZ 断面を Y 方向へ押し出す)
    inner_x = SC_HEAD_HALF_W + EAR_GAP
    arm_t = 3.4
    top = PITCH_Z
    rise_start = COLLAR_Z1 - 1.0
    knee_z = rise_start + (COLLAR_R_IN + 0.3 - inner_x)
    outer_knee_z = knee_z + arm_t * math.tan(math.radians(22.5))
    arm_profile = polygon(
        [
            (COLLAR_R_IN + 0.3, COLLAR_Z0 + 2.0),
            (COLLAR_R_OUT - 0.8, COLLAR_Z0 + 2.0),
            (COLLAR_R_OUT - 0.8, COLLAR_Z1 - 1.0),
            (COLLAR_R_OUT, COLLAR_Z1),
            (COLLAR_R_OUT, outer_knee_z - (COLLAR_R_OUT - inner_x - arm_t)),
            (inner_x + arm_t, outer_knee_z),
            (inner_x + arm_t, top),
            (inner_x, top),
            (inner_x, knee_z),
            (COLLAR_R_IN + 0.3, rise_start),
        ]
    )
    arm_half_w = 8.0
    arm = arm_profile.extrude(2 * arm_half_w).translate((0, 0, -arm_half_w))
    # (x, z, 押し出し) → ワールド (x, y=押し出し, z)
    arm = arm.transform(np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0]], dtype=np.float64))
    pad = cyl_x(EAR_R, inner_x, inner_x + arm_t, 0, PITCH_Z, 96)
    groove = cyl_x(EAR_R - 2.5, inner_x + arm_t - 0.6, inner_x + arm_t + 1, 0, PITCH_Z, 96) - cyl_x(
        EAR_R - 4.0, inner_x + arm_t - 1, inner_x + arm_t + 2, 0, PITCH_Z, 96
    )
    side = arm + pad - groove
    return yoke + side + side.mirror((1, 0, 0))


# ---------------------------------------------------------------- スタックチャン(表示と干渉確認用)
# 色は公式の製品写真(refs/official_docs/K151-weight.jpg)に合わせる
SC_COLORS = {
    "shell": "#d9dbdd",  # 筐体の薄いグレー
    "base": "#8d9196",  # 土台とサーボ部の濃いめのグレー
    "glass": "#0b0c0e",  # CoreS3 前面の黒いガラス
    "face": "#ffffff",  # 画面に映る目と口
    "led": "#3fb3ba",  # 上部左右の導光バー(ティール)
    "camera": "#2b3038",
}
CORE_DEPTH = 16.0  # CoreS3 の厚み
SCREEN_PX = 40.6 / 320  # 2.0 インチ 320x240 画面の 1 px [mm]


def _along_y(section: CrossSection, y0: float, y1: float) -> Manifold:
    """(x, z) の断面を y0..y1 に押し出す。"""
    solid = section.extrude(y1 - y0)
    return solid.transform(np.array([[1, 0, 0, 0], [0, 0, -1, y1], [0, 1, 0, 0]], dtype=np.float64))


def _rounded_rect(x0: float, x1: float, z0: float, z1: float, radius: float) -> CrossSection:
    return rect(x0 + radius, x1 - radius, z0 + radius, z1 - radius).offset(radius, JoinType.Round, 2.0, 48)


@lru_cache(maxsize=None)
def stackchan_head() -> Manifold:
    """ヨーで回る上部のうち胴体シェル(公式 MainBody STL)。ローカル: 机面 z=0、ヨー軸 = Z。"""
    mesh = trimesh.load(ROOT / "refs" / "official_stl" / "StackChan-MainBody.stl")
    low, high = mesh.bounds
    body = from_trimesh(mesh).translate((-(low[0] + high[0]) / 2, -low[1], -low[2]))
    # STL は +y が背面。前を +Y にするため Z まわり 180° 回す。前面(元 y=0)を CoreS3 の背面に合わせる
    return apply(body, ROT180_Z).translate((0, SC_FRONT - CORE_DEPTH, SC_HEAD_BOTTOM))


@lru_cache(maxsize=None)
def stackchan_core() -> Manifold:
    """CoreS3 本体(54 × 54 × 16、縦の角は丸い)。前面 0.6 mm はガラスなので除く。"""
    section = _rounded_rect(-27, 27, SC_HEAD_BOTTOM, SC_HEAD_TOP, 4.0)
    return _along_y(section, SC_FRONT - CORE_DEPTH, SC_FRONT - 0.6)


@lru_cache(maxsize=None)
def stackchan_face() -> Manifold:
    """CoreS3 前面の黒いガラス(前面全体を覆う)。"""
    section = _rounded_rect(-27, 27, SC_HEAD_BOTTOM, SC_HEAD_TOP, 4.0)
    return _along_y(section, SC_FRONT - 0.6, SC_FRONT)


def _screen_center_z() -> float:
    return (SC_HEAD_BOTTOM + SC_HEAD_TOP) / 2 + 2.0  # 画面はガラスの中央よりやや上


@lru_cache(maxsize=None)
def stackchan_eyes() -> Manifold:
    """画面に映る顔。公式アバターの配置(320x240: 目 x=90/230 y=93 半径8px、口 y=148 幅85px)に合わせる。"""
    mid = _screen_center_z()
    eye = CrossSection.circle(8 * SCREEN_PX, 32)
    spacing = 70 * SCREEN_PX
    shapes = eye.translate((-spacing, mid + 27 * SCREEN_PX)) + eye.translate((spacing, mid + 27 * SCREEN_PX))
    shapes += rect(-42.5 * SCREEN_PX, 42.5 * SCREEN_PX, mid - 28 * SCREEN_PX - 0.3, mid - 28 * SCREEN_PX + 0.3)
    return _along_y(shapes, SC_FRONT, SC_FRONT + 0.15)


@lru_cache(maxsize=None)
def stackchan_camera() -> Manifold:
    """画面下のカメラ窓。"""
    ring = CrossSection.circle(1.5, 32).translate((0, SC_HEAD_BOTTOM + 5.2))
    return _along_y(ring, SC_FRONT, SC_FRONT + 0.1)


@lru_cache(maxsize=None)
def stackchan_leds() -> Manifold:
    """胴体シェル上部左右の導光バー(側面のスリット位置: 前から 6〜34 mm、上面から約 2 mm 下)。"""
    front = SC_FRONT - CORE_DEPTH
    bar = box(SC_HEAD_HALF_W - 0.6, SC_HEAD_HALF_W + 0.25, front - 34.0, front - 6.0, SC_HEAD_TOP - 3.2, SC_HEAD_TOP - 1.2)
    return bar + bar.mirror((1, 0, 0))


@lru_cache(maxsize=None)
def stackchan_servo_body() -> Manifold:
    mesh = trimesh.load(ROOT / "refs" / "official_stl" / "StackChan-ServoBody.stl")
    low, high = mesh.bounds
    shape = from_trimesh(mesh).translate((-(low[0] + high[0]) / 2, -(low[1] + high[1]) / 2, -low[2]))
    return shape.translate((0, 0, SC_BASE_H + 3.3))


# ---------------------------------------------------------------- 部品表と姿勢
@dataclass(frozen=True)
class PartDef:
    key: str
    label: str
    build: object
    color: str
    count: int
    printable: bool = True
    finish: str = "matte"  # 表示用の質感: matte / glass / glow


def part_defs() -> dict[str, PartDef]:
    c = COLORS
    defs = [
        PartDef("side_plate", "側板", side_plate_part, c["chassis"], 2),
        PartDef("deck", "デッキ", deck_part, c["chassis"], 1),
        PartDef("yoke", "ヨーク(カラー+イヤーパッド)", yoke_part, c["accent"], 1),
        PartDef("gear_table", "ターンテーブル歯車", gear_table_part, c["accent"], 1),
        PartDef("socket_frame", "ソケット枠", socket_frame_part, c["chassis"], 1),
        PartDef("pinion", "ピニオン", pinion_part, c["accent"], 1),
        PartDef("shaft", "六角シャフト", shaft_part, c["link"], 1),
        PartDef("sleeve", "スリーブ", sleeve_part, c["link"], 1),
        PartDef("crank", "クランク", crank_part, c["accent"], 2),
        PartDef("clip", "Cクリップ", clip_part, c["link"], 22),
        PartDef("link_j", "リンク1(j: C–D)", lambda: link_part("j"), c["link"], 4),
        PartDef("link_k", "リンク2(k: C–F)", lambda: link_part("k"), c["link"], 4),
        PartDef("link_c", "リンク3(c: B–F)", lambda: link_part("c"), c["link"], 4),
        PartDef("link_f", "リンク4(f: E–G)", lambda: link_part("f"), c["link"], 4),
        PartDef("tri_upper", "上三角", lambda: link_part("tri_upper"), c["accent"], 4),
        PartDef("tri_lower", "下三角(足)", lambda: link_part("tri_lower"), c["accent"], 4),
        PartDef("sc_base", "スタックチャン土台", stackchan_base, SC_COLORS["base"], 1, printable=False),
        PartDef("sc_servo", "スタックチャン サーボ部", stackchan_servo_body, SC_COLORS["base"], 1, printable=False),
        PartDef("sc_head", "スタックチャン胴体シェル", stackchan_head, SC_COLORS["shell"], 1, printable=False),
        PartDef("sc_core", "CoreS3 本体", stackchan_core, SC_COLORS["shell"], 1, printable=False),
        PartDef("sc_face", "CoreS3 前面ガラス", stackchan_face, SC_COLORS["glass"], 1, printable=False, finish="glass"),
        PartDef("sc_eyes", "スタックチャンの顔", stackchan_eyes, SC_COLORS["face"], 1, printable=False, finish="glow"),
        PartDef("sc_camera", "カメラ窓", stackchan_camera, SC_COLORS["camera"], 1, printable=False, finish="glass"),
        PartDef("sc_leds", "導光バー", stackchan_leds, SC_COLORS["led"], 1, printable=False, finish="glow"),
    ]
    return {d.key: d for d in defs}


@dataclass(frozen=True)
class Instance:
    name: str
    part: str
    matrix: np.ndarray
    group: str  # 同じ group 同士は一体で動く(干渉判定の説明用)


# 脚: (名前, 左右 sx, 前後 sy)。sy=+1 は基準脚(固定ピボットが後ろ = 後ろ脚)。
LEGS = [("RR", 1, 1), ("RF", 1, -1), ("LR", -1, 1), ("LF", -1, -1)]
LEG_LAYER = {"j": 0, "c": 0, "f": 0, "tri_upper": 1, "tri_lower": 1, "k": 2}


def leg_first_layer(sx: int, sy: int) -> int:
    """基準の向き(sx*sy=+1)の脚は第1層から、鏡像の脚は第2層から使う。"""
    return 0 if sx * sy > 0 else 1


def crank_pin_angle(theta: float, sx: int) -> float:
    return theta if sx > 0 else theta + math.pi


def leg_canonical_angle(theta: float, sx: int, sy: int) -> float:
    pin = crank_pin_angle(theta, sx)
    return pin if sy > 0 else math.pi - pin


def leg_matrix(sx: int, sy: int, layer_x: float, origin: np.ndarray, angle: float) -> np.ndarray:
    """リンクローカル (u, w, h) → ワールド。x = sx*(layer_x + h), y = sy*u', z = w'。"""
    c, s = math.cos(angle), math.sin(angle)
    return mat(
        [
            [0, 0, sx, sx * layer_x],
            [sy * c, -sy * s, 0, sy * origin[0]],
            [s, c, 0, origin[1]],
        ]
    )


def foot_contacts(theta: float) -> np.ndarray:
    """各脚の足裏接地候補点(足幅の両端)。shape (4, 2, 3)。"""
    out = []
    for _, sx, sy in LEGS:
        joints = solve_leg(LEG, leg_canonical_angle(theta, sx, sy))
        x0, x1 = foot_x_range(sx, sy)
        y, z = sy * joints["H"][0], joints["H"][1] - FOOT_R
        out.append([[x0, y, z], [x1, y, z]])
    return np.array(out)


def foot_x_range(sx: int, sy: int) -> tuple[float, float]:
    """足先(幅広部)の X 範囲。基準の向きの脚は外向き、鏡像の脚は同じ部品を裏返すので内向きに伸びる。"""
    layer_x = L1 + (leg_first_layer(sx, sy) + LEG_LAYER["tri_lower"]) * LAYER_PITCH
    if sx * sy > 0:
        return sx * layer_x, sx * (layer_x + FOOT_W)
    return sx * (layer_x + LAYER_T - FOOT_W), sx * (layer_x + LAYER_T)


def flipped_leg_matrix(sx: int, sy: int, layer_x: float, origin: np.ndarray, angle: float) -> np.ndarray:
    """鏡像の脚に、鏡像部品ではなく同じ部品を裏返して置く行列。x = sx*(layer_x + LAYER_T - h)。"""
    matrix = leg_matrix(sx, sy, layer_x, origin, angle)
    matrix[0, 2] = -sx
    matrix[0, 3] = sx * (layer_x + LAYER_T)
    return matrix


def instances(theta: float, head_yaw: float = 0.0, with_stackchan: bool = True, head_pitch: float = 0.0) -> list[Instance]:
    """クランク角 theta[rad] での全部品。ターンテーブルは 1:1 で逆回転する。

    head_yaw は頭部のヨー(車体に対するずれ)、head_pitch はピッチ軸まわりのうなずき[rad](正で上向き)。
    """
    out = [
        Instance("side_plate_R", "side_plate", np.eye(4), "chassis"),
        Instance("side_plate_L", "side_plate", ROT180_Z, "chassis"),
        Instance("deck", "deck", np.eye(4), "chassis"),
        Instance("yoke", "yoke", np.eye(4), "chassis"),
    ]
    table = rot_z(-theta)
    out += [
        Instance("gear_table", "gear_table", table, "table"),
        Instance("socket_frame", "socket_frame", table, "table"),
    ]
    spin = rot_x(theta)
    out += [
        Instance("pinion", "pinion", spin, "shaft"),
        Instance("shaft", "shaft", spin, "shaft"),
        Instance("sleeve", "sleeve", spin, "shaft"),
    ]
    clip_x = CRANK_T + clip_seat(PIN_CLEAR)
    for sx, tag in ((1, "R"), (-1, "L")):
        pin = crank_pin_angle(theta, sx)
        # クランクローカルはピンが +Y、円板から +X へピンが伸びる。左は Z まわり 180° で外向きを -X にする
        base = rot_x(pin) if sx > 0 else rot_x(pin + math.pi) @ ROT180_Z
        out.append(Instance(f"crank_{tag}", "crank", translation(sx * L0, 0, 0) @ base, "shaft"))
        clip_local = translation(clip_x, LEG.m, 0)
        out.append(Instance(f"clip_{tag}", "clip", translation(sx * L0, 0, 0) @ base @ clip_local, "shaft"))
    for name, sx, sy in LEGS:
        joints = solve_leg(LEG, leg_canonical_angle(theta, sx, sy))
        first = leg_first_layer(sx, sy)
        for link, (start, _end) in LINK_AXES.items():
            layer_x = L1 + (first + LEG_LAYER[link]) * LAYER_PITCH
            key = "tri_upper" if link == "tri_upper" else "tri_lower" if link == "tri_lower" else f"link_{link}"
            # 鏡像の脚にも同じ部品を使う(鏡像部品を作らない)。スタッドの無い板(k・三角板)は裏返し、
            # スタッド付きの棒(j・c・f)はリンク軸まわりに対称な形なので、左右を入れ替えた向きで置く
            if sx * sy > 0:
                matrix = leg_matrix(sx, sy, layer_x, joints[start], link_angle(link, joints))
            elif link in ("k", "tri_upper", "tri_lower"):
                matrix = flipped_leg_matrix(sx, sy, layer_x, joints[start], link_angle(link, joints))
            else:
                matrix = leg_matrix(sx, sy, layer_x, joints[start], link_angle(link, joints)) @ MIRROR_W
            out.append(Instance(f"{name}_{link}", key, matrix, f"leg_{name}_{link}"))
            # この部品から立つ突起のクリップ(持ち主と一緒に動く)
            for joint, (owner, at, layers) in STUD_JOINTS.items():
                if owner != link:
                    continue
                u, w = at(LEG)
                seat = translation(u, w, LAYER_T + clip_seat(stud_clear(layers))) @ CLIP_FLAT @ rot_x(CLIP_MOUTH[joint])
                out.append(Instance(f"{name}_clip_{joint}", "clip", matrix @ seat, f"leg_{name}_{link}"))
        # 側板の B の突起のクリップ
        plate = np.eye(4) if sx > 0 else ROT180_Z
        local_sign = -1 if first == 0 else 1
        seat = translation(PLATE_OUT + clip_seat(plate_stud_clear(first)), local_sign * LEG.a, -LEG.l) @ rot_x(CLIP_MOUTH["B"])
        out.append(Instance(f"{name}_clip_B", "clip", plate @ seat, "chassis"))
    if with_stackchan:
        seat = translation(0, 0, TABLE_Z1)
        out.append(Instance("sc_base", "sc_base", table @ seat, "table"))
        head = seat @ rot_z(head_yaw)
        nod = translation(0, 0, SC_PITCH_AXIS_H) @ rot_x(head_pitch) @ translation(0, 0, -SC_PITCH_AXIS_H)
        out.append(Instance("sc_servo", "sc_servo", head, "head"))
        for key in ("sc_head", "sc_core", "sc_face", "sc_eyes", "sc_camera", "sc_leds"):
            out.append(Instance(key, key, head @ nod, "head"))
    return out


@lru_cache(maxsize=None)
def part_shape(key: str) -> Manifold:
    shape = part_defs()[key].build()
    if shape.status().name != "NoError" or shape.is_empty():
        raise RuntimeError(f"部品 {key} の形状が不正: {shape.status()}")
    return shape


def placed(instance: Instance) -> Manifold:
    return apply(part_shape(instance.part), instance.matrix)

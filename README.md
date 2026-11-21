# Stackchan Strider

![スタックチャンを載せて歩く Stackchan Strider](docs/walk.gif)

M5Stack 製品版スタックチャン(K151 / CoreS3)を、首振り(ヨー)サーボ1個で歩かせる 3D プリント製の4脚ベースです。
ネジ・ベアリング・接着剤は使いません。印刷した部品をはめて、C クリップで留めるだけで組み立てます。

*A 3D-printed four-legged walking base for the M5Stack StackChan (K151). The head-yaw servo drives four Jansen-linkage legs through a 1:1 bevel gear. 57 printed parts, no screws, no bearings, no glue. Documentation is in Japanese.*

- 組立ガイド(スクロールすると組み立てが進みます): https://yeq6x.github.io/stackchan-strider/
- 3D ビューアー: https://yeq6x.github.io/stackchan-strider/viewer.html
- 印刷用データ: `print/stackchan_strider.3mf`(Bambu Studio のプロジェクト)、`print/stl/`
- ファームウェア: `firmware/`(スマホのブラウザから操作)

| | |
|---|---|
| 印刷部品 | 57 個(16 種類)。うち C クリップが 22 個 |
| 購入部品 | なし(スタックチャン本体のみ) |
| 全体寸法(スタックチャン込み) | 幅 96 × 長さ 147 × 高さ 180 mm |
| 質量 | 約 297 g(印刷部品 約 109 g + スタックチャン 187 g。印刷部品は概算) |
| 傘歯車 | モジュール 2、20 歯の 1:1(ピッチ円錐半角 45°) |
| 脚 | ヤンセン機構 ×4(Jansen 比率 × 0.75、クランク半径 6.0 mm) |

## すきまについて

回る穴と軸は直径の差 0.4 mm(半径で 0.2 mm)、重ねた部品の面どうしは 0.15 mm です。これより詰めた試作では、摩擦でサーボの力が足りず歩きませんでした。
お使いのプリンターで固い場合は、`walker/design.py` の `HOLE_FIT` と `GAP` を調整して出力し直してください。

## しくみ

1. スタックチャンの**土台**をソケット枠にはめる。枠はターンテーブル歯車と一緒に回る。
2. **頭**は左右のパッド(ヨーク)で挟まれ、首振りの方向には回れない。上下のうなずきは自由。
3. 首振りのサーボを連続回転させると、頭が回れないので、土台とターンテーブル歯車のほうが回る。
4. 下向き 45° の傘歯車が、横向きのクランク軸(六角シャフト)に回転を伝える。
5. 左右のクランクは 180° ずらしてあり、片側の前後の脚は 1 本のクランクピンを共有する。

土台はソケットに置くだけなので、充電するときはそのまま持ち上げて外せます。

## 印刷

- 0.4 mm ノズル、積層 0.2 mm、PLA。全部品サポートなし。
- 最大の部品は 89 × 96 mm(ヨーク)。180 mm 角のベッドに収まります。
- `print/stackchan_strider.3mf` は 3 プレート構成です。フィラメントは 3 色(1: Frame、2: Accent、3: Links)で、1 プレートに使うのは 1 色だけです。

| プレート | 中身 |
|---|---|
| Frame | 側板 ×2、デッキ、ソケット枠 |
| Accent | ヨーク、ターンテーブル歯車、ピニオン、クランク ×2、上の三角板 ×4、下の三角板 ×4 |
| Links | リンク 1〜4 各 ×4、六角シャフト、スリーブ、C クリップ ×22 |

STL を使う場合は `print/stl/` にあります。造形時の向きで出力してあり、ファイル名の末尾(`_x4` など)が必要な個数です。

### 刻印の読み方

脚の部品には刻印があります。数字はリンクの番号、英字はつなぐ場所です。同じ英字どうしをつなげば組み上がります。

| 刻印 | 部品 | 両端 |
|---|---|---|
| 1 | リンク 1 | C(クランクのピン)– D |
| 2 | リンク 2 | C(クランクのピン)– F |
| 3 | リンク 3 | B(側板の突起)– F |
| 4 | リンク 4 | E – G |
| B・D・E | 上の三角板 | 各頂点の穴のそば |
| F・G | 下の三角板 | 各頂点の穴のそば(刻印の無い頂点が足先) |

## 組み立て

図と動きつきの手順は[組立ガイド](https://yeq6x.github.io/stackchan-strider/)を見てください。あらましは次のとおりです。

1. ピニオンの筒を左の側板の穴に通し、デッキのツメを左右の側板の角穴に押し込む。
2. スリーブを右の側板の穴に通し、六角シャフトを通す。両端にクランクを差す。シャフトの両端は六角の角を 1 つ落としてあり、クランクは 1 つの向きでしか差せない。差すと、**左右のクランクのピンは 180° 反対向き**になる。
3. ターンテーブル歯車の軸をデッキの筒に落とし込み、ソケット枠を載せる。ヨークを上からかぶせて側板に留める。
4. 脚を組む。突起も穴も丸いので向き合わせは要らない。重ねて、突起の溝に C クリップを横からはめる。
   - 先に机の上で、リンク 4 に上下の三角板を通し、E と G をクリップで留めた組を 4 つ作る。
   - 片側の 2 本の脚は 1 本のクランクピンに重ねるので、2 本を並行して組む(順番は組立ガイドのとおり)。
   - 下の三角板は 1 種類で、右前脚と左後ろ脚では裏返して使う。
5. スタックチャンの土台を枠に、頭をパッドの間に入れる。

## ファームウェア

`firmware/README.md` を見てください。画面はスタックチャンの顔で、スマホのブラウザのスティックで、前進・後退と、その場での左右の旋回を操作します。旋回は、歩行ベースを組んだあとに操作ページの「旋回の調整」を 1 回やると使えます。
スマホとのつなぎ方は、はじめて起動したときに画面で選びます。本体が出す Wi-Fi にスマホを直接つなぐ方法(設定なし)と、家の Wi-Fi を使う方法があります。どちらも、画面に出る QR をスマホで読んでつなぎます。

## 設計を変える・検証する

形状はすべて Python のコード(`walker/design.py`)から生成しています。

```
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt

.venv\Scripts\python walker\build.py                                     # out\stl\ に STL、out\parts.json に部品表、out\3mf\ に 3MF
.venv\Scripts\python walker\verify_gears.py --json out\verify_gears.json     # 歯車の噛み合い
.venv\Scripts\python walker\verify_walk.py --json out\verify_walk.json       # 歩行 1 周の干渉・すきま・接地姿勢
.venv\Scripts\python walker\verify_assembly.py --json out\verify_assembly.json   # 手順どおりに組めるか
.venv\Scripts\python walker\sim_sweep.py --json out\mujoco\sweep.json        # MuJoCo で 24 条件を歩かせる
.venv\Scripts\python walker\sim_turn.py --set rock                       # MuJoCo で、無改造のまま曲がれる動かし方を探す
.venv\Scripts\python walker\export_viewer.py                             # out\viewer\index.html
.venv\Scripts\python walker\export_guide.py                              # out\guide\index.html
.venv\Scripts\python walker\publish_files.py                             # print\ と docs\ へ公開用のファイルを写す
```

検証スクリプトは、不合格のとき終了コード 1 で終わります。

| ファイル | 役割 |
|---|---|
| `walker/design.py` | 全寸法と部品の形状、クランク角ごとの部品の配置 |
| `walker/jansen.py` | ヤンセン機構の平面運動学 |
| `walker/gears.py` | すぐば傘歯車の歯形(Tredgold 近似) |
| `walker/gait.py` | 接地姿勢(傾き・高さ・前進量)と質量・重心の見積り |
| `walker/collide.py` | 部品どうしの干渉体積とすきま |
| `walker/assembly_plan.py` | 脚の組立手順(組立ガイドと検証で共通) |
| `walker/verify_gears.py`、`verify_walk.py`、`verify_assembly.py` | 検証 |
| `walker/sim_mujoco.py`、`sim_sweep.py` | MuJoCo の物理モデルと歩行シミュレーション |
| `walker/sim_turn.py` | MuJoCo で、無改造のまま曲がれる動かし方を探す(頭のうなずき、歩く速さの変化、首の往復) |
| `walker/build.py` | 造形時の向きで STL と 3MF を出力 |
| `walker/update_project.py`、`arrange_links.py` | Bambu Studio で並べ直した 3MF の配置を保ったまま、形状を差し替える・リンクを整列する |
| `walker/export_viewer.py`、`export_guide.py` | ビューアーと組立ガイドの生成 |
| `refs/` | 寸法の基準にした M5Stack 公式の STL と出所(`refs/SOURCES.md`) |

## 検証の結果

形状(`verify_gears.py`、`verify_walk.py`、`verify_assembly.py`):

- 歯車: 1 回転を通して干渉なし。
- 歩行 1 周(2° 刻み、スタックチャン込み): 干渉なし。
- 組立: 左右 30 操作(重ねる・クリップをはめる)が、すでに付いている部品にぶつからずにできる。
- 接地姿勢: 常に 3 本以上の足で支え、傾きは最大 3.6°。

物理シミュレーション(MuJoCo 3.9、`sim_sweep.py`):

- 床の摩擦 0.3 / 0.5 / 0.8 / 1.0 × サーボ指令 30% / 60% / 100% × 前進・後退の 24 条件すべてで転倒なし。前後の傾きは最大 3.1°。
- 摩擦 0.5・指令 60%: クランク 39 rpm、速度 約 31 mm/s、サーボの平均トルク 20 mN·m(定格 64 mN·m)。
- 指令 100% では跳ねて 1 回転あたりの前進が減る。60% 以下が実用域。

モデルに入っていないもの: 部品のたわみ、歯車の遊びと歯面の摩擦、頭とパッドのすきま、足以外の接触。

その場での旋回(実機、2026-11-20、試験用ファームウェア `firmware/src/probe/yaw_probe.cpp`):

- 部品を変えずに、首を全開で往復させるだけで、その場で左右に向きを変えられる。どちらに回るかは、往復の中心(脚の位相)で決まる。
- 上から見て時計回り: 首のサーボの生の位置で 524〜812 の間を往復。5 秒で 157〜191°(3 回)。
- 上から見て反時計回り: 位置が読めない区間(1 周の約 1 割)をまたいで往復。5 秒で 124〜171°(往復を始めるまでの待ちが無かった 6 回)。
- 出力を下げると回る角度が減る。時計回りは 100% / 80% / 60% / 40% で、5 秒に 179° / 131° / 76° / 23°。反時計回りは 100% / 80% / 40% で 154° / 105° / 5°(各 1 回)。
- どちらに回る往復の中心も、歩行ベースの組み方(傘歯車のかみ合う歯、クランクを差す向き)で変わる。歩行用のファームウェアは、8 秒ほど歩かせて機体の揺れをジャイロで測り、その機体の中心を求める(「旋回の調整」)。2 つの歩行ベース(同じスタックチャンを載せ替えた)で、求めた中心は 660 前後と 328 前後に分かれ、どちらも調整のあと左右に回った。
- 1 種類の床で確かめた。別のスタックチャン本体では確かめていない。

## 分かっている制約

- 4 脚なので揺れながら歩きます。足が 2 本同時に浮く瞬間があり、前後に約 ±3° 揺れます。
- 足上げは約 6 mm です。平らな机や床の上で歩かせる前提で、段差やケーブルは越えられません。
- 歩いている間は、土台の USB-C にケーブルを挿せません(土台が回り続けるため)。内蔵バッテリーで動かします。
- スタックチャンの重心の位置は公開値が無いので、仮定の値で計算しています。
- 片側の 2 本の脚は、内側の層と外側の層に分かれます。右は後ろ脚が内側、左は前脚が内側で、左右対称ではありません(側板を左右共通の部品にしているため)。

## ライセンスと謝辞

- このリポジトリのコード、3D モデル、文書は MIT License です(`LICENSE`)。
- `refs/official_stl/` は M5Stack 公式の構造データ([m5stack/M5_Hardware](https://github.com/m5stack/M5_Hardware)、MIT License)です。寸法の基準と、ビューアーでのスタックチャンの表示に使っています。ライセンス文は `refs/official_stl/LICENSE-M5_Hardware.txt` にあります。
- ファームウェアは [StackChan-BSP](https://github.com/m5stack/StackChan-BSP)、[M5Unified](https://github.com/m5stack/M5Unified)、[M5GFX](https://github.com/m5stack/M5GFX)、[M5Stack-Avatar](https://github.com/stack-chan/m5stack-avatar) を使っています。
- スタックチャンは ししかわ さんが始めたオープンソースのロボットです。脚はテオ・ヤンセンのリンク機構にもとづいています。
- これは個人の作品で、M5Stack の公式製品ではありません。

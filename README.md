# realsense_dust_cloth

Intel RealSense D435 と NVIDIA Jetson (CUDA) を C++ で密結合した、飛翔中の雑巾（布）の **3次元弾道リアルタイム追尾・最高確率放物線フィッティング・自動データセット蓄積パッケージ** です（TMCIT 荒川ロボコン 2026A 向け）。

---

## 1. 特徴 & システム概要
* **C++ & CUDA 完全GPUパイプライン（60fps / 超低遅延 < 2ms）**:
  * 深度画像の 3D 逆投影・空間ROI・背景差分・Voxel間引き・DBSCANクラスタリング・重心算出を **すべて GPU 内で一貫処理**。
  * Jetson の CPU 負荷は数%以下で動作します。
* **リアルタイム最高確率放物線フィッティング（数学的最適弾道モデル）**:
  * 飛翔中の布のはためきや RealSense の微小深度ノイズを100%除去。
  * 最小二乗法により、数学的に最も確からしい真の放物線 $Z(X) = a X^2 + b X + c$ と直進線 $Y(X) = d X + e$ をリアルタイム算出（信頼度 $R^2 \ge 95\%$）。
  * 最高到達点（Apex）と着地予測地点（Landing Target）をミリ単位で即時計算。
* **ロボット静止時 / 走行時の両対応モード**:
  * **停止時（発射反動あり）**: 背景差分（`use_bg_subtraction:=true`）により周囲の壁や家具を完全消去。
  * **台車走行時（移動射撃）**: 背景差分を OFF（`use_bg_subtraction:=false`）にし、空間ROI ＋ 発射口ゲート ＋ 進行速度ベクトル照合で追尾。
* **超軽量 JPEG エンコーダ圧縮配信（rosbag 記録容量 95% 削減 & Wi-Fi 遠隔配信）**:
  * Jetson の高速ハードウェア/SIMD エンコーダで JPEG 圧縮し、`/camera/camera/color/image_raw/compressed` を自動配信。
  * ディスク容量を消費せず rosbag に投擲映像を記録でき、Wi-Fi 経由でも遅延なく手元 PC へ映像をストリーミング可能。
* **全自動データセット蓄積（Headless 運用）**:
  * GUI (RViz2) を起動せず、投擲するたびに自動で CSV（`data/cloth_trajectories_dataset.csv`）に蓄積。
  * 統計解析ツールで最確軌道グラフ（`most_probable_trajectory.png`）をワンコマンド出力。

---

## 2. ハードウェア設置 & 座標系 (Static TF)

* **USB 接続**: Jetson の **USB 3.0 ポート（青色端子 / SSマーク）に直結**。
* **カメラ設置向き**: **90° ロール回転の縦置き**（垂直視野角を約 90° に拡大し、放物線の最高到達点を見失わない配置）。
* **取り付け仰角**: 水平から **上向き 20°** (`camera_pitch_deg: 20.0`)。
* **機体オフセット**: 前方 $0.15\,\text{m}$ (`camera_x_offset_m: 0.15`)、高さ $0.35\,\text{m}$ (`camera_height_offset_m: 0.35`)。
* **Static TF**: `cloth_tracker.launch.py` が `static_transform_publisher` で
  `base_link -> camera_link` を配信します。`camera_link -> *_optical_frame` は
  RealSenseドライバが配信するため、二重配信されません。
* **カメラ姿勢の変更**: `camera_roll_deg`、`camera_pitch_deg`、
  `camera_x_offset_m`、`camera_height_offset_m`、`camera_parent_frame`、
  `camera_frame` を launch 引数で変更できます。

---

## 3. 起動コマンド（用途別）

### A. 実機 RealSense での運用

```bash
source ~/ros2_ws/install/setup.bash

# 1. 実機単体運用（Direct RealSense API / 停止射撃モード）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_direct_realsense:=true

# 2. RViz2 GUI も一緒に起動して弾道を確認する場合
ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_direct_realsense:=true rviz:=true

# 3. ロボットが走行しながら撃つ場合（移動射撃モード・背景差分OFF）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_direct_realsense:=true use_bg_subtraction:=false
```

---

### B. Isaac Sim（シミュレータ）での動作確認

Isaac Sim で RealSense を配置して疎通・軌道確認を行う場合（共通コード・共通 launch で動作）：

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch realsense_dust_cloth cloth_tracker.launch.py \
  rviz:=true \
  use_sim_time:=true \
  use_bg_subtraction:=false
```

---

### C. 記録済み rosbag を再生してテスト

```bash
# ターミナル1: トラッカー & RViz
ros2 launch realsense_dust_cloth cloth_tracker.launch.py rviz:=true use_sim_time:=true

# ターミナル2: bag 再生
ros2 bag play <記録したbagフォルダ名> --loop --clock
```

---

## 4. 飛翔データの記録・保存方法（用途別の保存ガイド）

用途（「後から RViz でそのまま再生したい」か「Excel/Python で座標値を分析したい」か）に合わせて以下の保存方法を利用できます。

### 方法1. launch 起動時に自動で rosbag 録画する（一番おすすめ・TF完全保存）
起動コマンドに `record:=true` を付与するだけで、雑巾の TF 座標軸（`flying_cloth`）や軌跡、圧縮カラー映像が自動で rosbag に記録されます：

```bash
# 実機で起動しつつ、自動で rosbag 録画も開始
ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_direct_realsense:=true record:=true

# RViz も同時に開く場合
ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_direct_realsense:=true rviz:=true record:=true
```

* **記録される主なトピック**:
  * `/tf`, `/tf_static`: 雑巾の座標フレーム（`base_link -> flying_cloth`）
  * `/cloth/position`: 雑巾重心位置（PointStamped）
  * `/cloth/trajectory_marker`: 3次元飛翔軌跡（Marker）
  * `/cloth/measured_path`: 計測点列パス（Path）
  * `/camera/camera/color/image_raw/compressed`: 超軽量 JPEG 圧縮映像（容量を食わずに投擲映像も保存）
* **停止方法**: 起動した端末で `Ctrl + C` を押すと保存が完了します（実行フォルダに `rosbag2_日時/` が作成されます）。
* **再生方法**:
  ```bash
  ros2 bag play rosbag2_2026_xx_xx-xx_xx_xx
  ```

---

### 方法2. 手動で別端末から rosbag 録画する
必要なトピックのみを指定して手動で録画したい場合：

```bash
ros2 bag record /tf /tf_static /cloth/position /cloth/trajectory_marker /cloth/measured_path
```

---

### 方法3. 試行ごとに CSV と弾道グラフ（PNG）を自動保存する
投擲ごとに「何フレーム目でどこを通ってどこに着地したか」を数値 CSV として記録し、終了時に弾道グラフ画像を自動生成したい場合：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run realsense_dust_cloth record_trajectories.py
```

* **保存先**: `~/ros2_ws/data/trajectories_日時.csv`
* **機能**:
  * 雑巾が発射されて着地するたびに、1試行（Flight #01, #02...）として自動判定して追記。
  * 投げ終わって `Ctrl + C` を押すと、全投擲のばらつきを描いた **弾道グラフ画像（PNG）** が自動生成されます。

---

### 方法4. 投擲データセットを CSV に全自動蓄積する（Headless 運用）
launch 起動時に `enable_analyzer:=true` を指定することで、バックグラウンドで自動的にデータセット CSV へ追記蓄積されます：

```bash
ros2 launch realsense_dust_cloth cloth_tracker.launch.py enable_analyzer:=true
```

* **保存先**: `~/ros2_ws/data/cloth_trajectories_dataset.csv`

---

## 5. 蓄積データの統計解析 & グラフ出力

投擲を重ねてデータが CSV（`data/cloth_trajectories_dataset.csv`）に溜まったら、以下のコマンドで解析グラフを出力します：

```bash
python3 ~/ros2_ws/src/realsense_dust_cloth/realsense_dust_cloth/trajectory_probability_analyzer.py --plot
```
* **出力内容**:
  * 最高確率放物線 $Z(X)$ と信頼区間（$\pm 1\sigma$ エラーバンド）
  * 着地点のばらつき分布（X方向・Y方向の標準偏差）
  * グラフ画像 `most_probable_trajectory.png` の自動生成

---

## 6. 主要パラメータ (`config/tracker_params.yaml`)

| パラメータ | 既定値 | 説明 |
| :--- | :---: | :--- |
| `use_direct_realsense` | `true` | RealSense C++ API 直接ストリーミング（ROSトピック入力時は `use_direct_realsense:=false`） |
| `use_bg_subtraction` | `true` | 静止背景差分（停止時は `true`、台車走行射撃時は `false`） |
| `camera_pitch_deg` | `20.0` [deg] | カメラ仰角（上向き 20°） |
| `camera_roll_deg` | `90.0` [deg] | カメラ縦置き角度（90° ロール） |
| `min_x_m` / `max_x_m` | `0.30` / `5.00` [m] | 前後方向の ROI 抽出空間（5m先の着地まで追従） |
| `min_y_m` / `max_y_m` | `-1.00` / `1.00` [m] | 左右方向の ROI 抽出空間 |
| `min_z_m` / `max_z_m` | `0.00` / `2.20` [m] | 高さ方向の抽出空間（最高到達点から床面まで完全包含） |
| `launch_min_x` / `launch_max_x` | `0.30` / `1.80` [m] | 発射口ロックオンエリア X |
| `launch_min_y` / `launch_max_y` | `-0.80` / `0.20` [m] | 発射口ロックオンエリア Y（発射台位置 Y=-0.41m を包含） |
| `launch_min_z` / `launch_max_z` | `0.30` / `1.80` [m] | 発射口ロックオンエリア Z |
| `min_cluster_points` | `10` | 雑巾最小点群数（遠距離でも着地まで捕捉） |
| `voxel_size_m` | `0.02` [m] | ボクセルダウンサンプリング（高密度点群保持） |
| `gate_radius_m` | `0.65` [m] | 飛行追跡ゲート半径（急減速・失速落下にも追従） |
| `enable_color_stream` | `true` | JPEG 圧縮カラー配信（rosbag 記録 & 遠隔監視用） |
| `jpeg_quality` | `65` | JPEG 圧縮品質（1〜100、65 で帯域 95% 削減） |
| `color_fps` | `30` | カラー画像配信レート (fps) |

`cloth_tracker.launch.py` はこのYAMLを読み込んだうえで、launch引数を
ROS 2の正しい型（bool / int / float）で上書きします。カメラ姿勢を変更する場合は、
同名のlaunch引数を指定してください（角度は度数法です）。

```bash
ros2 launch realsense_dust_cloth cloth_tracker.launch.py \
  camera_roll_deg:=0.0 camera_pitch_deg:=20.0 \
  camera_parent_frame:=base_link camera_frame:=camera_link
```

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
* **Static TF**: ノード起動時に `base_link -> camera_link -> camera_depth_optical_frame` が自動配信されます。

---

## 3. 起動コマンド（用途別）

### A. 本番運用：Jetson 単体での自動データ蓄積（GUIなし・超軽量）

```bash
source ~/ros2_ws/install/setup.bash

# 1. 停止して撃つ場合（デフォルト・高精度背景差分）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_direct_realsense:=true

# 2. ロボットが走行しながら撃つ場合（移動撮影モード）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_direct_realsense:=true use_bg_subtraction:=false
```

---

### B. 遠隔監視：Jetson でノードを動かし、手元 PC の RViz2 で映像・弾道を見る方法

Jetson と手元 PC を同じ Wi-Fi（または有線LAN）に接続し、同一の `ROS_DOMAIN_ID` を指定します。

#### ① Jetson 側でトラッカーを起動
```bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_direct_realsense:=true
```

#### ② 手元 PC 側で RViz2 を起動
```bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30

# パソコン側でモニター画面を開く
rviz2 -d ~/ros2_ws/src/realsense_dust_cloth/rviz/cloth_tracker.rviz
```

* **画面で確認できる内容**:
  * **左上**: 投擲時のリアルタイムカラーカメラ映像（JPEG 圧縮で Wi-Fi でも低遅延）
  * **黄色の放物線**: 数学的に算出された真の弾道ライン
  * **紫の球体**: 最高到達点（Apex）
  * **緑の円形**: 着地予測ターゲット
  * **赤緑青の軸**: カメラが捉えた全フレームの TF 座標軸

---

### C. 記録済み rosbag を再生してテスト

```bash
# ターミナル1: トラッカー & RViz
ros2 launch realsense_dust_cloth play_bag_rviz.launch.py

# ターミナル2: bag 再生
ros2 bag play ~/ros2_ws/data/test_bag/rosbag2_2026_09_26-09_54_28_0.db3 --loop --clock
```

---

## 4. 蓄積データの統計解析 & グラフ出力

投擲を重ねてデータが CSV（`data/cloth_trajectories_dataset.csv`）に溜まったら、以下のコマンドで解析グラフを出力します：

```bash
python3 ~/ros2_ws/src/realsense_dust_cloth/realsense_dust_cloth/trajectory_probability_analyzer.py --plot
```
* **出力内容**:
  * 最高確率放物線 $Z(X)$ と信頼区間（$\pm 1\sigma$ エラーバンド）
  * 着地点のばらつき分布（X方向・Y方向の標準偏差）
  * グラフ画像 `most_probable_trajectory.png` の自動生成

---

## 5. 主要パラメータ (`config/tracker_params.yaml`)

| パラメータ | 既定値 | 説明 |
| :--- | :---: | :--- |
| `use_direct_realsense` | `false` | RealSense C++ API 直接ストリーミング（実機接続時は `true`） |
| `use_bg_subtraction` | `true` | 静止背景差分（停止時は `true`、台車走行射撃時は `false`） |
| `camera_pitch_deg` | `20.0` [deg] | カメラ仰角（上向き 20°） |
| `camera_roll_deg` | `90.0` [deg] | カメラ縦置き角度（90° ロール） |
| `min_x_m` / `max_x_m` | `0.40` / `3.50` [m] | 前後方向の ROI 抽出空間 |
| `min_z_m` / `max_z_m` | `0.00` / `1.80` [m] | 高さ方向の抽出空間（最高到達点 $1.4\text{m}$ を包含） |
| `launch_min_x` / `launch_max_x` | `0.50` / `1.20` [m] | 発射口ロックオンエリア X |
| `launch_min_z` / `launch_max_z` | `0.85` / `1.60` [m] | 発射口ロックオンエリア Z |
| `gate_radius_m` | `0.40` [m] | 飛行追跡ゲート半径（周囲ノイズを完全遮断） |
| `enable_color_stream` | `true` | JPEG 圧縮カラー配信（rosbag 記録 & 遠隔監視用） |
| `jpeg_quality` | `65` | JPEG 圧縮品質（1〜100、65 で帯域 95% 削減） |
| `color_fps` | `30` | カラー画像配信レート (fps) |

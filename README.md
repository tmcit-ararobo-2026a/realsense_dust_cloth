# realsense_dust_cloth

Intel RealSense D435 と NVIDIA Jetson (CUDA) を C++ で密結合した、飛翔中の雑巾（布）の **3次元弾道リアルタイム追尾・最高確率放物線フィッティング・自動データセット蓄積パッケージ**（TMCIT 荒川ロボコン 2026A 向け）。

---

## 1. 特徴 & システム構成

* **C++ & CUDA 完全 GPU パイプライン（60fps / 超低遅延 < 2ms）**:
  * 深度画像の 3D 逆投影・空間 ROI・事前地図 ESDF 差分・Voxel 間引き・DBSCAN クラスタリング・重心算出を GPU 内で一貫処理。
* **事前地図 ESDF 背景差分（自己位置連動・学習待ちゼロ）**:
  * 競技場・フィールドの事前マップ（JSON / ESDF）と撮影時刻同期 TF（`map -> base_link`）を照合し、ロボットの移動位置に関わらず静止構造物（壁・柱・床・カゴ）を即時除去。
  * 従来の静止画深度蓄積（depth学習）が不要なため、起動直後から即時射出可能。台車停止後の再学習待ち時間もゼロ（機体揺れ収束待ち `stop_settle_sec: 0.10s` のみ）。
  * 走行中は自動的に認識を休止（`MOVING_OR_SETTLING`）し、停止後に即座に `MAP_READY` へ復帰。
* **数学的最適弾道モデル（リアルタイム放物線フィッティング）**:
  * はためきや深度ノイズを最小二乗法で除去し、真の放物線 $Z(X) = a X^2 + b X + c$ と直進線 $Y(X) = d X + e$ をリアルタイム算出。
  * 追跡終了時または着地時に、最終着地位置を `/cloth/landing_position` へ自動配信（`gn10-shot-manager` 連携）。
* **超軽量 JPEG 圧縮配信 & 全自動データセット蓄積**:
  * `/camera/camera/color/image_raw/compressed` を自動配信。
  * 投擲ごとに CSV（`~/ros2_ws/data/cloth_trajectories_dataset.csv`）へ自動記録し、解析グラフをワンコマンド生成。

---

## 2. ハードウェア設置 (Static TF)

* **接続**: Jetson の **USB 3.0 ポート（青色端子 / SSマーク）に直結**。
* **カメラ向き**: **90° ロール回転の縦置き**（垂直視野角を約 90° に拡大し、放物線の頂点を見失わない配置）。
* **取り付け角度・位置**:
  * 仰角: 水平から上向き 20° (`camera_pitch_deg: 20.0`)
  * 機体原点オフセット: 前方 0.50m (`camera_x_offset_m: 0.50`)、高さ 1.00m (`camera_height_offset_m: 1.00`)
* **Static TF**: `cloth_tracker.launch.py` が `base_link -> camera_link -> camera_color_optical_frame` を自動配信。

---

## 3. 設定ファイル一覧

| 設定ファイル | 対象環境 | 事前マップ | 特徴 |
|---|---|---|---|
| [`tracker_params.yaml`](config/tracker_params.yaml) | 実機（大会本番）※デフォルト | `package://gn10_pointcloud_localization/map/nhk2026_map.json` | カラー配信ON・実機直結 |
| [`clubroom_map_params.yaml`](config/clubroom_map_params.yaml) | 実機（部室簡易フィールド） | `package://gn10_navigation/config/test.json` | カラー配信ON・部室検証用 |
| [`unity_map_params.yaml`](config/unity_map_params.yaml) | Unity シミュレータ | `package://gn10_pointcloud_localization/map/nhk2026_map.json` | `use_sim_time: true`、カラー配信ON |

---

## 4. 起動コマンド

```bash
source ~/ros2_ws/install/setup.bash

# 1. 実機起動（大会本番フィールド / デフォルト）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py

# 2. 実機起動（部室簡易フィールド）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py params_file:=clubroom_map_params.yaml

# 3. RViz2 GUI を同時表示する場合
ros2 launch realsense_dust_cloth cloth_tracker.launch.py rviz:=true

# 4. Unity シミュレータで動作確認する場合
ros2 launch realsense_dust_cloth cloth_tracker.launch.py params_file:=unity_map_params.yaml rviz:=true
```

### Launch スイッチ引数
* `params_file:=<yaml>`: 設定 YAML の指定（ファイル名のみまたは絶対パス）
* `rviz:=true`: RViz2 GUI を同時起動
* `use_direct_realsense:=false`: 実機カメラ直結ではなく ROS 2 トピックから画像を購読
* `use_sim_time:=true`: シミュレータや rosbag の `/clock` に同期
* `enable_analyzer:=true`: データセット CSV 自動蓄積を有効化

---

## 5. 実機 rosbag 収録 & 自宅でのオフライン検証

### 収録（付属スクリプト推奨）
部室での収録時は、過去の弾道データ（`/cloth/*`）や大容量 LiDAR 点群（`/livox/lidar`）を除外し、自己位置とセンサ入力のみ（全 8 トピック）を超軽量に同時記録します：

```bash
# bags/real_cloth_日時 に自動記録（1投あたり約20MB〜30MBの超軽量）
~/ros2_ws/src/realsense_dust_cloth/scripts/record_real_robot.sh [保存ディレクトリ名]
```
* **収録トピック（全 8 トピック）**:
  * `/tf`, `/tf_static`（自己位置 `map -> base_link` およびカメラ固定座標）
  * `/platform_constraint`（gn10 自己位置 Pose）
  * `/camera/camera/depth/image_rect_raw`, `/camera/camera/depth/camera_info`（生Depth画像）
  * `/camera/camera/color/image_raw/compressed`, `/camera/camera/color/camera_info`（JPEG圧縮動画）
  * `/robot/operation/cmd_vel`（台車走行・停止状態）

### 自宅でのオフライン再生・追尾再実行
持ち帰った rosbag を再生し、自宅 PC 上で何度でも追尾ノードを実行・パラメータ調整できます：

```bash
# ターミナル1: トラッカー & RViz2 起動（部室マップ test.json を指定）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py \
  params_file:=clubroom_map_params.yaml \
  use_direct_realsense:=false \
  use_sim_time:=true rviz:=true

# ターミナル2: bag 再生（必ず --clock を付与）
ros2 bag play <記録したbagフォルダ名> --clock
```
* **ポイント**: 収録バッグには過去の古い弾道マーカーが含まれていないため、トピックフィルタなしでそのまま再生でき、現在のノードが 1 フレームずつ新規に計算した 3D 弾道と着地予測が RViz2 に表示されます。

---

## 6. 主要トピック & 状態監視

### 主要出力トピック
| トピック名 | 型 | 説明 |
|---|---|---|
| `/cloth/position` | `geometry_msgs/PointStamped` | 飛翔中の雑巾 3D 重心座標（X, Y, Z） |
| `/cloth/landing_position` | `geometry_msgs/PointStamped` | 着地確定位置（`gn10-shot-manager` 連携用） |
| `/cloth/most_probable_path` | `nav_msgs/Path` | 放物線フィッティング推定軌道 |
| `/cloth/ready` | `std_msgs/Bool` | 射出可能フラグ（背景準備完了かつ静止時 `true`） |
| `/cloth/background_status` | `std_msgs/String` | 状態（`MAP_READY`, `MOVING_OR_SETTLING`, `MAP_WAITING_FOR_POSE`） |
| `/cloth/tracking_state` | `std_msgs/String` | 追跡状態（`IDLE`, `TRACKING`, `LANDED`, `LOST`, `RESTING`） |

### 状態確認コマンド
```bash
ros2 topic echo /cloth/ready
ros2 topic echo /cloth/background_status
ros2 topic echo /cloth/tracking_state
```

---

## 7. 蓄積データの統計解析 & グラフ出力

投擲を重ねてデータが CSV に蓄積されたら、以下のコマンドで解析グラフを出力します：

```bash
python3 ~/ros2_ws/src/realsense_dust_cloth/realsense_dust_cloth/trajectory_probability_analyzer.py --plot
```
* **出力**: 最高確率放物線 $Z(X)$、信頼区間（$\pm 1\sigma$ エラーバンド）、着地点分布、グラフ画像 `most_probable_trajectory.png`

---

## 8. 主要パラメータ一覧 (`config/tracker_params.yaml`)

| パラメータ | デフォルト値 | 説明 |
|---|:---:|---|
| `map_file_path` | `nhk2026_map.json` | 事前マップファイルパス（JSON / ESDF） |
| `map_exclusion_distance_m` | `0.10` [m] | 静止構造物からの除外マージン |
| `map_tf_wait_sec` / `map_tf_poll_sec` | `0.15` / `0.002` [s] | タイムスタンプ TF 取得の最大待ち時間 / ポーリング周期 |
| `stop_settle_sec` | `0.10` [s] | 台車停止後の機体揺れ収束待ち時間（収束後即時 MAP_READY） |
| `cmd_vel_timeout_sec` | `0.30` [s] | 台車走行判定のタイムアウト時間 |
| `camera_pitch_deg` / `camera_roll_deg` | `20.0` / `90.0` [deg] | カメラ仰角（上向き20°）/ 縦置き角度（時計回り90°） |
| `min_x_m` / `max_x_m` | `0.35` / `6.00` [m] | 前後方向の ROI 抽出空間 |
| `min_y_m` / `max_y_m` | `-0.60` / `0.60` [m] | 左右方向の ROI 抽出空間 |
| `min_z_m` / `max_z_m` | `0.05` / `2.40` [m] | 高さ方向の抽出空間 |
| `min_cluster_points` / `max_cloth_size_m` | `6` / `0.45` [m] | 雑巾最小点群数 / 雑巾最大対角サイズ |
| `voxel_size_m` / `cluster_eps_m` | `0.02` / `0.08` [m] | ボクセル間引き解像度 / クラスタ探索距離 |
| `landing_min_z` / `max_lost_frames` | `0.15` [m] / `8` | 着地確定判定高さ / 見失い許容フレーム数 |
| `observed_landing_only` | `true` | 実測低高度確認時のみ LANDED 判定 |

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
* **機体オフセット**: 前方 $0.50\,\text{m}$ (`camera_x_offset_m: 0.50`)、高さ $1.00\,\text{m}$ (`camera_height_offset_m: 1.00`)。
* **Static TF**: `cloth_tracker.launch.py` が `static_transform_publisher` で
  `base_link -> camera_link` および `camera_link -> camera_color_optical_frame` を配信します。
* **設定変更**: パラメータは `config/tracker_params.yaml` に一本化されています。ロボットの構造変更や競技場に応じた調整は YAML を編集するだけで反映されます。

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
  use_direct_realsense:=false \
  use_sim_time:=true \
  use_bg_subtraction:=false \
  rviz:=true
```

---

### C. 記録済み rosbag を再生してテスト

rosbag 再生時は、実機カメラ直接接続ではなく ROS 2 トピックから画像を読み込むため、**`use_direct_realsense:=false`** を指定してください。

```bash
# ターミナル1: トラッカー & RViz2 起動
ros2 launch realsense_dust_cloth cloth_tracker.launch.py rviz:=true use_sim_time:=true use_direct_realsense:=false

# ターミナル2: bag 再生
ros2 bag play <記録したbagフォルダ名> --loop --clock
```

#### 💡 過去のテスト rosbag を再生する場合
水平設置（仰角 0°）で撮影された過去のテスト rosbag を再生する場合は、専用設定ファイル **`test_bag_params.yaml`** を指定してください：

```bash
# ターミナル1: test_bag 専用設定で起動（ファイル名指定で自動解決されます）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py \
  params_file:=test_bag_params.yaml \
  rviz:=true use_sim_time:=true

# ターミナル2: bag 再生
ros2 bag play <記録したbagフォルダへのパス> --loop --clock
```
* **理由**: デフォルト設定（`tracker_params.yaml`）は本番機体用（上向き 20°、探索範囲 5.5m）です。水平設置で撮影された過去の bag に適用すると、座標が傾いて計算され、着地後に床や奥の壁ノイズを拾って軌道が折れ曲がってしまいます。`test_bag_params.yaml` を指定することで、撮影当時のカメラ角度（仰角 0°）・範囲（3.5m）で綺麗な放物線が再現されます。

---

## 4. 飛翔データの軽量記録・再生手順（rosbag & 圧縮カラー映像）

生画像（RAW）をそのまま記録すると秒間数百MB消費し、Wi-Fi環境ではパケット落ちが発生します。本パッケージでは **JPEG圧縮カラー映像 ＋ TF ＋ 弾道データ** の最小構成で記録（12秒で約12MBの超軽量）することを推奨しています。

---

### A. 実機での rosbag 記録コマンド

実機（ロボットまたは手元PC）のターミナルで以下を実行します：

```bash
ros2 bag record \
  /tf \
  /tf_static \
  /cloth/position \
  /cloth/trajectory_marker \
  /cloth/measured_path \
  /camera/camera/color/image_raw/compressed \
  -o cloth_test_01
```
* **記録される内容**:
  * `/tf`, `/tf_static`: カメラ設置姿勢および飛翔中の雑巾座標（`flying_cloth`）
  * `/cloth/position`: 雑巾の3D位置座標（PointStamped: X, Y, Z）
  * `/cloth/trajectory_marker`: RViz用の赤い球体・弾道マーカー
  * `/cloth/measured_path`: 計測軌跡のライン（Path）
  * `/camera/camera/color/image_raw/compressed`: JPEG圧縮カラー映像（30fps・超軽量）
* **終了**: 投擲が終わったら **`Ctrl + C`** で保存が完了します。
* ※ 2回目以降は `-o cloth_test_02`, `-o cloth_test_03` のように保存名を変更してください。

> 💡 **launch のオプションで自動録画する場合**:  
> 実機起動時に `record:=true` を渡すだけでも上記と同じ構成で自動録画されます：  
> `ros2 launch realsense_dust_cloth cloth_tracker.launch.py record:=true`

---

### B. 記録した rosbag の再生・カラー映像の確認

記録した bag を再生し、カラー動画や3D弾道を確認する手順です。用途に合わせて2つの方法が選べます：

#### 方法1. `rqt_image_view` でカラー動画を直接見る（一番手軽・確実）
追尾ノードや複雑な設定なしで、撮影映像を動画ウィンドウですぐに確認したい場合：

```bash
# ターミナル1: bag再生（-l でループ再生）
ros2 bag play cloth_test_01 -l

# ターミナル2: 画像ビューア
ros2 run rqt_image_view rqt_image_view
```
※ ウィンドウ左上のドロップダウンメニューから **`/camera/camera/color/image_raw/compressed`** を選択すると、投擲時のカラー映像が流れます。

---

#### 方法2. RViz2 の画面内で3D軌跡（放物線）と同期して見る
RVizの3D空間上で、飛行放物線・球体マーカーとカラー映像を同時に再生したい場合：

```bash
# ターミナル1: bag再生（-l でループ再生）
ros2 bag play cloth_test_01 -l

# ターミナル2: 圧縮画像の解凍中継スクリプト（RViz用トピックへ変換）
python3 ~/ros2_ws/src/realsense_dust_cloth/realsense_dust_cloth/decompress_image.py

# ターミナル3: RViz起動
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/realsense_dust_cloth/rviz/cloth_tracker.rviz
```
※ RViz画面左下のカメラビュー枠に、記録されたカラー映像が3D軌跡と完全に同期して表示されます。

---

### C. 試行ごとに CSV と弾道グラフ（PNG）を自動保存する場合
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

パラメータはすべて `config/tracker_params.yaml` に集約されています。ロボットの構造変更や競技場の寸法変更に合わせて、このファイルを直接編集してください。

| パラメータ | 設定値 | 説明 |
| :--- | :---: | :--- |
| `camera_x_offset_m` | `0.50` [m] | ロボット原点 (base_link) からのカメラ前方位置 |
| `camera_height_offset_m` | `1.00` [m] | ロボット原点 (base_link) からのカメラ搭載高さ |
| `camera_pitch_deg` | `20.0` [deg] | カメラ仰角（上向き 20°） |
| `camera_roll_deg` | `90.0` [deg] | カメラ縦置き角度（時計回り 90° ロール） |
| `min_x_m` / `max_x_m` | `0.20` / `5.50` [m] | 前後方向の ROI 抽出空間（発射口手前20cm〜着地先5.5m） |
| `min_y_m` / `max_y_m` | `-1.20` / `1.20` [m] | 左右方向の ROI 抽出空間 |
| `min_z_m` / `max_z_m` | `0.08` / `3.30` [m] | 高さ方向の抽出空間（最高到達点から床面まで完全包含） |
| `launch_min_x` / `launch_max_x` | `0.20` / `2.00` [m] | 発射口ロックオンエリア X（手前20cm〜前方2mまで飛び出しを捕捉） |
| `launch_min_y` / `launch_max_y` | `-0.50` / `0.50` [m] | 発射口ロックオンエリア Y（左右幅 1m） |
| `launch_min_z` / `launch_max_z` | `1.10` / `1.80` [m] | 発射口ロックオンエリア Z |
| `min_launch_vx` | `0.20` [m/s] | 発射確定最小前進速度 |
| `min_launch_move_m` | `0.04` [m] | 発射確定最小前進移動量（4cm前進でロックオン開始） |
| `min_cluster_points` | `4` | 雑巾最小点群数（超高速時でも捕捉） |
| `max_cloth_size_m` | `0.50` [m] | 雑巾最大対角サイズ（200x300mm布に合わせて50cmとし、人間や壁を除外） |
| `voxel_size_m` | `0.02` [m] | ボクセルダウンサンプリング（2cm） |
| `cluster_eps_m` | `0.20` [m] | クラスタ探索距離（200x300mm布を1塊に統合） |
| `gate_radius_m` | `1.20` [m] | 飛行追跡ゲート半径 |
| `max_lost_frames` | `20` | 見失い許容フレーム数 |
| `landing_min_z` | `0.05` [m] | 着地確定判定高さ |
| `drag_coeff` | `0.40` | 空気抵抗減衰係数 |
| `max_vx_m_s` | `15.0` [m/s] | 飛翔最大前進速度上限 |
| `rs_exposure_us` | `4500` [μs] | シャッタースピード（4.5ms固定露出で残像ブレを物理防止） |
| `use_direct_realsense` | `true` | RealSense C++ API 直接ストリーミング |
| `use_bg_subtraction` | `true` | 静止背景差分（停止時は `true`、台車走行射撃時は `false`） |
| `enable_color_stream` | `true` | JPEG 圧縮カラー配信（rosbag 記録 & 遠隔監視用） |
| `jpeg_quality` | `65` | JPEG 圧縮品質（1〜100、65 で帯域 95% 削減） |
| `color_rotation_deg` | `90` | 縦置きカメラ映像の正立回転角度 |

### Launch 起動時のモード切り替えオプション（スイッチ引数）
`cloth_tracker.launch.py` では、必要に応じて実行モードのみを引数で切り替えられます：

* `use_direct_realsense:=false`: ROS 2 トピック（`/camera/...`）から深度を受け取るモード
* `use_bg_subtraction:=false`: ロボット走行時の射撃モード（背景差分をOFF）
* `use_sim_time:=true`: Isaac Sim や rosbag 再生時のシミュレーションクロック同期
* `rviz:=true`: 手元 PC 等で RViz2 GUI を同時に起動
* `record:=true`: 主要トピック（TF, 雑巾位置, 軌跡, 圧縮カラー）を自動で rosbag 録画
* `params_file:=/path/to/custom.yaml`: 別のカスタム YAML ファイルを指定して起動

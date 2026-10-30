# realsense_dust_cloth

Intel RealSense D435 と NVIDIA Jetson (CUDA) を C++ で密結合した、飛翔中の雑巾（布）の **3次元弾道リアルタイム追尾・最高確率放物線フィッティング・自動データセット蓄積パッケージ**（TMCIT 荒川ロボコン 2026A 向け）。

---

## 1. 特徴 & システム構成

* **C++ & CUDA による深度画像・点群処理**:
  * 深度画像の 3D 逆投影・空間 ROI 抽出・CUDA DBSCAN クラスタリング・重心算出を GPU 内で高速処理。
* **自己位置推定 ESDF 動的背景除去連携**:
  * 自己位置推定ノード（`gn10_pointcloud_localization`）の 3D ESDF 距離場と密連携。
  * トラッカーが ROI クロップ点群（`/realsense/cropped_cloud`）および予測マーカー（`/cloth/filter_prediction`）を発行し、自己位置ノード側で静的マップ（床・壁・障害物）を除去した `/realsense/dynamic_cloud` を受信して追跡。
* **数学的最適弾道モデル（リアルタイム放物線フィッティング）**:
  * はためきや深度ノイズを最小二乗法で除去し、真の放物線 $Z(X) = a X^2 + b X + c$ と直進線 $Y(X) = d X + e$ をリアルタイム算出。
  * `observed_landing_only: true` では実測点が着地高さ（$Z \le 0.15\text{m}$）に到達して `LANDED` に遷移した瞬間に `/cloth/landing_position` を発行。
* **超軽量 JPEG 圧縮配信 & 全自動データセット蓄積**:
  * `/camera/camera/color/image_raw/compressed` を自動配信。
  * 投擲ごとに CSV（`data/cloth_trajectories_dataset.csv`）へ自動記録し、解析グラフをワンコマンド生成。

```
[RealSense D435] ──(Direct librealsense2 API)──> [cloth_tracker_cuda_node]
                                                        │
                                                        ▼ クロップ点群 (/realsense/cropped_cloud)
                                            [gn10_pointcloud_localization] (ESDF背景除去)
                                                        │
                                                        ▼ 動的抽出点群 (/realsense/dynamic_cloud)
                                                 [CUDA DBSCAN & 弾道推定]
                                                        │
                                                        ▼ 着地座標 (/cloth/landing_position)
```

---

## 2. ハードウェア設置 (Static TF)

* **接続**: Jetson の **USB 3.0 ポート（青色端子 / SSマーク）に直結**。
* **カメラ向き**: **90° ロール回転の縦置き**（垂直視野角を約 90° に拡大し、放物線の頂点を見失わない配置）。
* **取り付け角度・位置**:
  * 仰角: 水平から上向き 20° (`camera_pitch_deg: 20.0`)
  * 機体原点オフセット: 前方 0.50m (`camera_x_offset_m: 0.50`)、高さ 1.00m (`camera_height_offset_m: 1.00`)
* **Static TF**: `publish_static_tf: true` のとき、トラッカーノードから固定カメラ TF（`base_link -> camera_link`）を自動配信。

---

## 3. 設定ファイル一覧 (YAML 一元管理)

すべての動作パラメータは `config/` ディレクトリの YAML ファイルで一元管理されています。

| 設定ファイル | 対象環境 | Direct RealSense | `use_sim_time` | 特徴 |
|---|---|:---:|:---:|---|
| [`localization_cloud_params.yaml`](config/localization_cloud_params.yaml) | **実機（Jetson Orin）※デフォルト** | `true` | `false` | librealsense2 C++ API によるハードウェア直結・ゼロ引数起動 |
| [`unity_localization_cloud_params.yaml`](config/unity_localization_cloud_params.yaml) | **Unity シミュレータ** | `false` | `true` | ROS 2 トピック購読・シミュレータ時刻同期 (`sim:=true`) |

---

## 4. 起動コマンド

### 実機起動（ゼロ引数起動）
実機（Jetson Orin）では追加引数は一切不要です。

```bash
# ターミナル 1: 自己位置推定ノード
source ~/ros2_ws/install/setup.bash
ros2 launch gn10_pointcloud_localization localization.launch.py

# ターミナル 2: 雑巾トラッカー
source ~/ros2_ws/install/setup.bash
ros2 launch realsense_dust_cloth cloth_tracker.launch.py
```

### Unity シミュレーション起動
Unity シミュレータ使用時は `sim:=true` を指定します。

```bash
# ターミナル 1: 自己位置推定ノード (シミュレーション時刻)
source ~/ros2_ws/install/setup.bash
ros2 launch gn10_pointcloud_localization localization.launch.py use_sim_time:=true

# ターミナル 2: 雑巾トラッカー (シミュレーションプロファイル自動選択)
source ~/ros2_ws/install/setup.bash
ros2 launch realsense_dust_cloth cloth_tracker.launch.py sim:=true rviz:=true
```

### 主な Launch オプション引数
* `sim:=true`: Unity シミュレーション環境への切り替え（`unity_localization_cloud_params.yaml` を自動選択）
* `rviz:=true`: 追跡用 RViz2 設定ファイルを同時起動
* `params_file:=<ファイル名>`: 任意のカスタム YAML を読み込む場合に指定
* `enable_analyzer:=false`: 軌跡 CSV 自動記録を無効化する場合に指定

---

## 5. 主要 ROS 2 トピック一覧

### 入力トピック
* `/realsense/dynamic_cloud` (`sensor_msgs/msg/PointCloud2`): 自己位置推定ノードから送られる背景除去済み動的点群
* `/realsense/filter_status` (`std_msgs/msg/String`): 自己位置フィルタの状態 (`READY` 等)
* `/robot/operation/cmd_vel` (`geometry_msgs/msg/Twist`): ロボット台車の走行速度指令（停止後 0.1s 判定に使用）
* `/camera/camera/color/image_raw` (`sensor_msgs/msg/Image`): カラー画像 (Unity/rosbag時)

### 出力トピック
* `/realsense/cropped_cloud` (`sensor_msgs/msg/PointCloud2`): 自己位置ノードへ送信する ROI 抽出点群
* `/cloth/filter_prediction` (`visualization_msgs/msg/Marker`): 自己位置ノードへ送信する雑巾保護予測マーカー
* `/cloth/position` (`geometry_msgs/msg/PointStamped`): 現在の雑巾重心座標
* `/cloth/landing_position` (`geometry_msgs/msg/PointStamped`): **最終着地予測座標**（着地時のみ発行）
* `/cloth/tracking_state` (`std_msgs/msg/String`): 追跡状態 (`WAITING` / `TRACKING` / `LANDED` / `LOST`)
* `/cloth/ready` (`std_msgs/msg/Bool`): 追跡準備完了フラグ
* `/camera/camera/color/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`): JPEG 圧縮カラー動画

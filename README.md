# realsense_dust_cloth

Intel RealSense D435 と NVIDIA Jetson (CUDA) を C++ で密結合した、飛翔中の雑巾（布）の **3次元弾道リアルタイム追尾・最高確率放物線フィッティング・自動データセット蓄積パッケージ**（TMCIT 荒川ロボコン 2026A 向け）。

---

## 1. 特徴 & システム構成

* **C++ & CUDA による深度画像・点群処理**:
  * 深度画像の 3D 逆投影・空間 ROI・事前地図 ESDF 差分・Voxel 間引き・DBSCAN クラスタリング・重心算出を GPU 内で一貫処理。
* **事前地図 ESDF 背景差分（自己位置連動・学習待ちゼロ）**:
  * 競技場・フィールドの事前マップ（JSON / ESDF）と撮影時刻同期 TF（`map -> base_link`）を照合し、ロボットの移動位置に関わらず静止構造物（壁・柱・床・カゴ）を即時除去。
  * 従来の静止画深度蓄積（depth学習）が不要なため、深度背景を蓄積する待ち時間は不要。ただしカメラ情報・画像時刻に対応する TF の取得が必要で、移動後は停止待ち `stop_settle_sec: 0.10s` を設ける。
  * 走行中は自動的に認識を休止（`MOVING_OR_SETTLING`）し、停止待ちが終わり、画像時刻・TF が有効なら `MAP_READY` へ復帰。停止判定は走行指令に基づき、実際の揺れを測定するものではない。
* **数学的最適弾道モデル（リアルタイム放物線フィッティング）**:
  * はためきや深度ノイズを最小二乗法で除去し、真の放物線 $Z(X) = a X^2 + b X + c$ と直進線 $Y(X) = d X + e$ をリアルタイム算出。
  * `observed_landing_only: true` では実測点が着地高さの条件を満たして `LANDED` になった場合に `/cloth/landing_position` を配信。見失い (`LOST`) や静止判定 (`RESTING`) は着地成功として扱わない。
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
* **Static TF**: `publish_static_tf: true` のとき、追跡ノードが設定されたカメラ固定 TF を配信。取付値は実機に合わせ、同じ TF の二重配信を避ける。

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
* `enable_analyzer:=true`: データセット CSV 自動蓄積を有効化（デフォルト `true`。不要なら `false`）

数値や通常設定は選んだ YAML を基準にする。`use_sim_time` と `use_direct_realsense` は、launch 引数で明示指定したときだけ YAML を上書きする。

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
| `/cloth/measured_path` | `nav_msgs/Path` | 実測点をつないだ軌道 |
| `/cloth/ready` | `std_msgs/Bool` | 準備状態（処理可能かつ追跡中でないとき `true`。射出を自動禁止する信号ではない） |
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
| `map_exclusion_distance_m` | `0.08` [m] | 静止構造物からの除外マージン |
| `map_tf_wait_sec` / `map_tf_poll_sec` | `0.15` / `0.002` [s] | タイムスタンプ TF 取得の最大待ち時間 / ポーリング周期 |
| `stop_settle_sec` | `0.10` [s] | 台車停止後の機体揺れ収束待ち時間（指令に基づく停止待ち） |
| `cmd_vel_timeout_sec` | `0.25` [s] | 台車走行判定のタイムアウト時間 |
| `camera_pitch_deg` / `camera_roll_deg` | `20.0` / `90.0` [deg] | カメラ仰角（上向き20°）/ 縦置き角度（時計回り90°） |
| `min_x_m` / `max_x_m` | `0.60` / `6.00` [m] | 前後方向の ROI 抽出空間 |
| `min_y_m` / `max_y_m` | `-0.60` / `0.60` [m] | 左右方向の ROI 抽出空間 |
| `min_z_m` / `max_z_m` | `0.05` / `3.10` [m] | 高さ方向の抽出空間 |
| `min_cluster_points` / `max_cloth_size_m` | `45` [点] / `0.80` [m] | 雑巾最小点群数 / 雑巾最大対角サイズ |
| `voxel_size_m` / `cluster_eps_m` | `0.02` / `0.10` [m] | ボクセル間引き解像度 / クラスタ探索距離 |
| `landing_min_z` / `max_lost_frames` | `0.15` [m] / `8` | 着地確定判定高さ / 見失い許容フレーム数 |
| `observed_landing_only` | `true` | 実測低高度確認時のみ LANDED 判定 |


## 9. 処理の流れとコードの読み方

現在のブランチでは、localization が使う地図と同じ内容のファイルを追跡側も読み込み、独立した GPU 地図を保持する。localization の GPU メモリや `/esdf_map` を直接共有しているわけではない。自己位置は撮影時刻の `map -> base_link` TF で受け取り、RealSense 点群を自己位置推定には投入しない。

深度画像1フレームの処理順は次のとおり（現在の実装順）。

1. カメラ情報・走行指令・画像時刻・撮影時刻の TF を確認する。移動中や必要情報がない間は処理を休止する。
2. 深度の距離・近傍の連続性を検査し、点をロボット座標へ変換する。
3. 事前地図との距離で背景を除外する。追跡が確定した飛翔体の予測位置付近には除外を緩める保護がある。
4. ロボット基準の空間 ROI と、射出位置／追跡中の予測位置による探索範囲を適用する。
5. Voxel で間引き、点をクラスタにまとめ、点数・サイズなどから候補を選ぶ。
6. 候補の移動量・速度・予測位置を使って状態と軌道を更新し、実測位置などを配信する。

地図差分は「地図から離れた点」を残す処理であり、動いている点だけを確定するものではない。地図にない静止物も残るため、後段の候補選択と追跡が必要。

| 読みたい内容 | ファイル・主な入口 |
|---|---|
| カメラ受信、停止判定、全体の呼び出し、出力 | [`src/cloth_tracker_cuda_node.cpp`](src/cloth_tracker_cuda_node.cpp)：`process_and_publish` |
| 地図設定、画像時刻と TF の検査、状態配信 | [`src/map_tracking_control.cpp`](src/map_tracking_control.cpp)：`prepare_map_frame` |
| JSON / ESDF の読み込み、GPU 地図の生成 | [`src/map_background.cpp`](src/map_background.cpp) |
| 地図差分、ROI、Voxel、クラスタ処理 | [`src/cuda_kernels.cu`](src/cuda_kernels.cu)：`process_depth_frame_cuda` |
| 射出判定、予測、追跡終了条件 | [`src/ballistic_tracker.cpp`](src/ballistic_tracker.cpp) |
| 実機・Unity の調整値 | [`config/`](config/) 内の起動時に指定した YAML |

## 10. 実機調整で確認する設定と表示

Jetson では通常の launch を起動し、手元の PC では RViz のみを開く。PC でも追跡ノードを起動すると二重になるため、以下を使う。

```bash
# Jetson（localization は通常の手順で起動済み）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py

# 手元の PC（このパスにリポジトリがある場合）
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/realsense_dust_cloth/rviz/cloth_tracker.rviz
```

PC と Jetson の ROS 通信設定を合わせ、PC の `ros2 topic list` に `/cloth/...` が見えることを確認する。実機は `use_sim_time: false`、Unity は `true`。Unity は `unity_map_params.yaml` で起動する。

| 調整したい内容 | YAML の項目 |
|---|---|
| 実機カメラ接続・時刻 | `use_direct_realsense`, `use_sim_time` |
| カメラ取付位置・向き | `camera_x_offset_m`, `camera_height_offset_m`, `camera_pitch_deg`, `camera_roll_deg` |
| 地図と有効範囲 | `map_file_path`, `map_frame`, `map_min_*_m`, `map_max_*_m`, `map_reject_unknown` |
| 背景除外の距離 | `map_exclusion_distance_m`, `map_tracking_keep_radius_m` |
| 空間 ROI | `min_x_m`〜`max_z_m`（`base_link` 基準） |
| 射出時の探索範囲 | `launch_min_*`, `launch_max_*` |
| 点数・クラスタ条件 | `voxel_size_m`, `cluster_eps_m`, `min_cluster_points`, `min_tracking_points`, `min_cloth_size_m`, `max_cloth_size_m` |
| 停止待ち・指令監視 | `enable_cmd_vel_watch`, `cmd_vel_topic`, `cmd_vel_timeout_sec`, `stop_settle_sec` |
| ROI・点群の表示 | `publish_debug_cloud` |

YAML の変更後はノードを再起動する。調整中は変更値と試射結果を記録する。

`publish_debug_cloud: true` で `/cloth/debug/roi_markers` と確認用点群を配信する。ROI の枠は表示だけを無効にしてもフィルタ自体は有効。点群の GPU→CPU 転送と通信が増えるため、確認後に `false` に戻して負荷を比較する。

`/cloth/debug/filtered_points` は地図差分・ROI・探索範囲・Voxel 処理後の点であり、カメラの全点や地図差分だけの結果ではない。少数点で早期終了したフレームでは表示が空になることもある。`/cloth/debug/cluster_points` は追跡中の候補を表示し、未追跡時は消去されるので、空の表示だけから原因を断定しない。

| 状態・症状 | 最初に確認すること |
|---|---|
| `MAP_WAITING_FOR_CAMERA_INFO` | 深度カメラ情報の受信 |
| `MAP_WAITING_FOR_POSE` | 撮影時刻の `map -> base_link` TF、localization の時刻 |
| `MAP_IMAGE_TIME_INVALID` | 実機／Unity の `use_sim_time`、画像の遅延 |
| `MOVING_OR_SETTLING` | 走行指令と停止待ち |
| `MAP_PROCESSING_ERROR` | 地図・GPU のエラーログ |
| `MAP_READY` だが `IDLE` のまま | ROI、射出範囲、残った点、候補の点数・大きさ |
| 途中で `LOST` | 視野外、遠距離の点数不足、壁や床付近での背景除外 |
| 地図と壁が重ならない | 地図の内容、自己位置、カメラ取付値 |

`/esdf_map` と MID360 の生点群は、RViz に追加すると地図と自己位置の照合に使える。ただし `/esdf_map` の表示を追加しても、追跡側が差分に使う地図は変更されない。違う実験場所では対応する地図を選び、PCD を使う場合は追跡側用に対応する ESDF を準備する（追跡側の直接入力は JSON / ESDF）。

Jetson 全体の負荷と最後まで追える割合は実機で測定する。カメラ設定の FPS は処理性能の保証ではない。

## 11. localization側で地図差分する実験モード（別ブランチ）

`tracking_input_mode: external_dynamic_cloud` では、RealSenseの深度からROI・Voxel処理後の点群を `/realsense/cropped_cloud` に送り、gn10 localizationの `/realsense/dynamic_cloud` を受け取ってクラスタ選択・予測追跡します。追跡側のESDFは読み込まず、地図と自己位置はlocalization側で管理します。RealSense点群は自己位置の探索には使用しません。

両リポジトリの対応する新方式ブランチをビルドして使用します。

```bash
# 実機：通常のMID360・IMUの起動後、localizationを1つだけ起動
ros2 launch gn10_pointcloud_localization localization.launch.py \
  params_file:=/absolute/path/to/gn10-pointcloud-localization/config/realsense_filter_params.yaml
# 別ターミナル：RealSenseを開く追跡ノードも1つだけ起動
ros2 launch realsense_dust_cloth cloth_tracker.launch.py \
  params_file:=localization_cloud_params.yaml enable_analyzer:=false
```

Unityではlocalization用YAMLを `unity_realsense_filter_params.yaml` にし、localizationのlaunchへ `use_sim_time:=true` を指定します。追跡側は次の設定で起動します。

```bash
ros2 launch realsense_dust_cloth cloth_tracker.launch.py \
  params_file:=unity_localization_cloud_params.yaml enable_analyzer:=false rviz:=true
```

既存の引数なし起動は従来どおり追跡側で地図差分する方式です。新方式へ自動で切り替わりません。実機での負荷・追跡率の確認後に通常運用への採用を判断します。

確認トピック：`/realsense/filter_status`、`/cloth/background_status`、`/cloth/tracking_state`。準備完了は `EXTERNAL_READY`、入力途絶は `EXTERNAL_RESULT_TIMEOUT`。空の点群も観測として扱い、入力が途切れた場合はwatchdogで見失いを処理します。

追跡確定後は予測位置の球を `/cloth/filter_prediction`（Marker）に配信します。深度の撮影時刻と同じstampで、ADDは保護あり、DELETEは保護なしを表します。localizationは同じ時刻の点群と対応付け、球内の点を地図差分で消さずに残します。球の半径は追跡側YAMLの `external_tracking_keep_radius_m` で管理します。

地図差分で消えた点を追跡側で復元することはできません。また、保護範囲の静的な点も残るため、壁・床付近で誤った塊に移らないか確認が必要です。点群の往復転送と待ち時間を含めた軽量化は実測が必要です。

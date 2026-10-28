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
ros2 launch realsense_dust_cloth cloth_tracker.launch.py

# 2. RViz2 GUI も一緒に起動して弾道を確認する場合
ros2 launch realsense_dust_cloth cloth_tracker.launch.py rviz:=true

# 3. ロボットが走行しながら撃つ場合（移動射撃モード・背景差分OFF）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py use_bg_subtraction:=false
```

---

### B. Isaac Sim（シミュレータ）での動作確認

Isaac Sim や Gazebo 等のシミュレータ上で動作確認を行う場合、以下の最小 3 オプションで実機同様に起動できます：

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch realsense_dust_cloth cloth_tracker.launch.py \
  use_sim_time:=true \
  use_direct_realsense:=false \
  rviz:=true
```
* **オプションの役割**:
  * `use_sim_time:=true`: シミュレータのクロック（`/clock`）に同期。
  * `use_direct_realsense:=false`: 実機 USB ではなくシミュレータの ROS 2 画像トピックを購読。
  * `rviz:=true`: 弾道確認用の RViz2 画面を表示。
  * ※ 背景差分（`use_bg_subtraction`）は実機同様デフォルトで有効（`true`）となるため、シミュレータ内の静止した床や壁も自動で消去されます。Isaac Sim 固有の `32FC1`（float深度）もノード側で自動判別・変換されます。

---

### C. 記録済み rosbag を再生してテスト（1コマンド自動再生）

付属の `play_bag_rviz.launch.py` を使うと、バッグ再生・TF 発行・ノード起動・RViz 画面表示がワンコマンドで自動実行されます：

```bash
source ~/ros2_ws/install/setup.bash

# デフォルト（テストバッグ cloth_test_11_camera_only をループ再生）
ros2 launch realsense_dust_cloth play_bag_rviz.launch.py

# 任意の bag フォルダを指定する場合
ros2 launch realsense_dust_cloth play_bag_rviz.launch.py bag_path:=<bagフォルダへのパス>
```

#### （参考）ターミナルを分けて手動再生する場合

実機カメラ直接接続ではなく ROS 2 トピックから画像を読み込むため、**`use_direct_realsense:=false`** を指定してください。

```bash
# ターミナル1: トラッカー & RViz2 起動
ros2 launch realsense_dust_cloth cloth_tracker.launch.py rviz:=true use_sim_time:=true use_direct_realsense:=false

# ターミナル2: bag 再生（カメラ・Depthトピックのみに限定して再生）
ros2 bag play <記録したbagフォルダ名> --loop --clock --topics \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/color/camera_info
```

> 💡 **なぜ `--topics` を指定するのか？**  
> rosbag に過去の軌道トピック（`/cloth/*`）や TF（`/tf`）が記録されている場合、オプションなしで再生すると過去の古い弾道表示が混入し、現在起動中のノードの追尾計算と競合してしまいます。  
> `--topics` でカメラ映像と生Depthのみに限定して再生することで、純粋なセンサ入力だけを流し、**現在起動中のノードに新規で追尾・弾道計算を行わせる実験**が可能になります（過去の失敗原因などを当時の軌道のまま確認したい場合は `--topics` を外して全トピック再生してください）。

#### 💡 過去のテスト rosbag を再生する場合
水平設置（仰角 0°）で撮影された過去のテスト rosbag を再生する場合は、専用設定ファイル **`test_bag_params.yaml`** を指定してください：

```bash
# ターミナル1: test_bag 専用設定で起動（ファイル名指定で自動解決されます）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py \
  params_file:=test_bag_params.yaml \
  rviz:=true use_sim_time:=true

# ターミナル2: bag 再生（カメラ・Depthトピックのみ限定）
ros2 bag play <記録したbagフォルダへのパス> --loop --clock --topics \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/color/camera_info
```
* **理由**: デフォルト設定（`tracker_params.yaml`）は本番機体用（上向き 20°、探索範囲 5.5m）です。水平設置で撮影された過去の bag に適用すると、座標が傾いて計算され、着地後に床や奥の壁ノイズを拾って軌道が折れ曲がってしまいます。`test_bag_params.yaml` を指定することで、撮影当時のカメラ角度（仰角 0°）・範囲（3.5m）で綺麗な放物線が再現されます。

---

## 4. 飛翔データの軽量記録・再生手順（rosbag & 圧縮カラー映像）

生画像（RAW）をそのまま記録すると秒間数百MB消費し、Wi-Fi環境ではパケット落ちが発生します。本パッケージでは **JPEG圧縮カラー映像 ＋ TF ＋ 弾道データ** の最小構成で記録（12秒で約12MBの超軽量）することを推奨しています。

---

### A. 実機での rosbag 記録コマンド（生Depth・カラー・弾道データ完全版）

実機（ロボットまたは手元PC）のターミナルで以下を実行します。**生Depth画像とカメラ内部パラメータ（CameraInfo）を含めることで、持ち帰ったPC上で何度でも追尾ノードを再実行・パラメータ最適化が可能**になります（全15トピック）：

```bash
ros2 bag record \
  /tf \
  /tf_static \
  /cloth/position \
  /cloth/trajectory_marker \
  /cloth/measured_path \
  /cloth/most_probable_path \
  /cloth/probability_markers \
  /cloth/predicted_landing_point \
  /cloth/debug/roi_markers \
  /cloth/debug/cluster_points \
  /cloth/debug/filtered_points \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/color/camera_info \
  -o cloth_test_04
```
* **記録される主要トピックの内訳**:
  * `/camera/camera/depth/image_rect_raw`, `/camera/camera/depth/camera_info`: **生Depth画像とカメラ情報**（`use_direct_realsense: true` 実機直結時でも録画時のみ自動配信。オフライン再解析・アルゴリズム検証に必須）
  * `/camera/camera/color/image_raw/compressed`: JPEG圧縮カラー映像（30fps・正立回転済み・超軽量）
  * `/camera/camera/color/camera_info`: カラーカメラの内部パラメータ情報
  * `/tf`, `/tf_static`: カメラ設置姿勢および飛翔中の雑巾座標（`flying_cloth`）
  * `/cloth/position`: 雑巾の3D位置座標（PointStamped: X, Y, Z）
  * `/cloth/trajectory_marker`: RViz用の赤い球体・弾道マーカー
  * `/cloth/measured_path`: 計測軌跡のライン（Path）
  * `/cloth/most_probable_path`: 最小二乗フィッティングされた放物線軌道
  * `/cloth/probability_markers`: 放物線の最高到達点・着地予測マーカー
  * `/cloth/predicted_landing_point`: 算出された地面への着地予測座標
  * `/cloth/debug/roi_markers`: 空間ROI（緑枠）・発射口ゲート（水色枠）・追尾ゲート球（黄球）
  * `/cloth/debug/cluster_points`: そのフレームで「雑巾」として抽出された3D点群（追跡途切れの原因特定用）
  * `/cloth/debug/filtered_points`: 背景差分・ROIを通過した全3D点群（壁ノイズや点群密度の検証用）
* **終了**: 投擲が終わったら **`Ctrl + C`** で保存が完了します。
* ※ 2回目以降は `-o cloth_test_05`, `-o cloth_test_06` のように保存名を変更してください。

> 💡 **容量を極力抑えたい場合（超軽量モード）**:  
> Depth画像が不要で投擲結果と動画だけを残したい場合は、上記のリストから `/camera/camera/depth/...` の2行を抜くことで、1投擲あたり約10MBの超軽量で保存できます。

> 💡 **launch のオプションで自動録画する場合**:  
> 実機起動時に `record:=true` を渡すことでも、生Depthを含む全トピックが自動録画されます：  
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

#### 方法2. RViz2 の画面内で3D軌跡（放物線）と同期して見る（記録当時の軌道レビュー）
当時の飛行放物線・球体マーカーとカラー映像をそのまま再生したい場合（全トピックをそのまま出力）：

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

#### 方法3. 追尾アルゴリズムの再実験・パラメータ検証（カメラ・Depthのみフィルタ再生）
bag 内に記録されている前回の軌道マーカー（`/cloth/*`）や TF を遮断し、純粋なセンサ入力のみを流して新パラメータで追尾し直したい場合：

```bash
# ターミナル1: トラッカー & RViz2 起動
ros2 launch realsense_dust_cloth cloth_tracker.launch.py \
  params_file:=test_bag_params.yaml \
  rviz:=true use_sim_time:=true

# ターミナル2: カメラ・Depthトピックのみに限定して再生（--topics）
ros2 bag play cloth_test_01 --loop --clock --topics \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/color/camera_info
```
※ 前回の古い弾道や座標軸が混ざることなく、現在のノードが1フレームずつ新パラメータで追尾した結果のみを観察できます。

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
| `min_x_m` / `max_x_m` | `0.35` / `3.50` [m] | 前後方向の ROI 抽出空間（発射直後からシャッター壁手前3.5m） |
| `min_y_m` / `max_y_m` | `-0.60` / `0.60` [m] | 左右方向の ROI 抽出空間 |
| `min_z_m` / `max_z_m` | `0.05` / `2.40` [m] | 高さ方向の抽出空間（天井梁ノイズを遮断） |
| `launch_min_x` / `launch_max_x` | `0.35` / `1.80` [m] | 発射口ロックオンエリア X（飛び出しを即時捕捉） |
| `launch_min_y` / `launch_max_y` | `-0.45` / `0.45` [m] | 発射口ロックオンエリア Y |
| `launch_min_z` / `launch_max_z` | `0.80` / `2.10` [m] | 発射口ロックオンエリア Z |
| `min_launch_vx` | `0.30` [m/s] | 発射確定最小前進速度 |
| `min_launch_move_m` | `0.05` [m] | 発射確定最小前進移動量（静止ノイズ誤認識防止） |
| `min_cluster_points` | `6` | 雑巾最小点群数（高速飛翔で薄くなっても捕捉） |
| `max_cloth_size_m` | `0.45` [m] | 雑巾最大対角サイズ（フライングピクセルや奥の尾を除外） |
| `voxel_size_m` | `0.02` [m] | ボクセルダウンサンプリング（2cm） |
| `cluster_eps_m` | `0.08` [m] | クラスタ探索距離（遠方ノイズとの鎖状結合を遮断） |
| `continuity_thresh_m` | `0.05` [m] | 輪郭連続性閾値（5cm以上の奥行き急変をフライングピクセルとして遮断） |
| `continuity_min_neighbors` | `3` | 有効近傍ピクセル数（密集した塊のみ抽出） |
| `gate_radius_m` | `0.65` [m] | 飛行追跡ゲート半径（壁・天井へのワープ乗り移り防止） |
| `max_lost_frames` | `12` | 見失い許容フレーム数 |
| `landing_min_z` | `0.15` [m] | 着地確定判定高さ（カゴ・床面到達でLANDEDへ遷移） |
| `drag_coeff` | `0.40` | 空気抵抗減衰係数 |
| `max_vx_m_s` | `15.0` [m/s] | 飛翔最大前進速度上限 |
| `use_direct_realsense` | `true` | RealSense C++ API 直接ストリーミング |
| `use_bg_subtraction` | `true` | 静止背景差分（停止時は `true`、台車走行射撃時は `false`） |
| `enable_color_stream` | `true` | JPEG 圧縮カラー配信（rosbag 記録 & 遠隔監視用） |
| `jpeg_quality` | `65` | JPEG 圧縮品質（1〜100、65 で帯域 95% 削減） |
| `color_rotation_deg` | `90` | 縦置きカメラ映像の正立回転角度 |

### Launch 起動時のモード切り替えオプション（スイッチ引数）
`cloth_tracker.launch.py` では、必要に応じて実行モードのみを引数で切り替えられます：

* `use_direct_realsense:=false`: ROS 2 トピック（`/camera/...`）から深度を受け取るモード（実機ではデフォルトで `true`）
* `use_bg_subtraction:=false`: ロボット走行時の射撃モード（背景差分をOFF）
* `use_sim_time:=true`: Isaac Sim や rosbag 再生時のシミュレーションクロック同期
* `rviz:=true`: 手元 PC 等で RViz2 GUI を同時に起動
* `record:=true`: 主要トピック（TF, 雑巾位置, 軌跡, 圧縮カラー）を自動で rosbag 録画
* `params_file:=/path/to/custom.yaml`: 別のカスタム YAML ファイルを指定して起動


## 追加：地図差分と停止後背景学習の切り替え

従来方式は `background_mode: depth` のまま利用できる。地図・自己位置は不要。
`background_mode: map` は事前マップと撮影時刻の `map -> base_link` TF を使い、背景画像を学習せず静止構造物を除く。
どちらも射出カウンタは不要。走行中は認識を休止し、停止後の揺れ待ちはYAMLで設定する。

| 設定ファイル | 用途 |
|---|---|
| `tracker_params.yaml` | 従来の実機・停止後背景学習 |
| `test_bag_params.yaml` | 従来の固定カメラbag。地図・自己位置不要 |
| `unity_depth_params.yaml` | Unityで従来方式を比較 |
| `map_tracker_params.yaml` | 実機の事前マップ差分 |
| `unity_map_params.yaml` | Unityの事前マップ差分 |

Unity側を再生し、既存の自己位置推定がTFを出している状態で起動する。追跡ノードを二重起動しない。

```bash
source ~/ros2_ws/install/setup.bash
# 事前マップ差分
ros2 launch realsense_dust_cloth cloth_tracker.launch.py params_file:=unity_map_params.yaml enable_analyzer:=false rviz:=true
# 地図なしの従来方式（上の追跡ノードを終了してから）
ros2 launch realsense_dust_cloth cloth_tracker.launch.py params_file:=unity_depth_params.yaml enable_analyzer:=false rviz:=true
```

実機の地図方式は `params_file:=map_tracker_params.yaml`。通常の引数なし起動は従来方式。
launchの `use_sim_time / use_direct_realsense / use_bg_subtraction` は未指定ならYAMLを尊重する。明示指定したときだけ上書きする。

### 地図方式の設定と条件

- `map_file_path` は自己位置推定と同じ地図を指定する。gn10のJSON（BOX / CYLINDER）と `.esdf` バイナリに対応。`package://パッケージ名/相対パス` が使える。VISUAL_BOXはgn10と同様に距離場に含めない。
- PCDを使う場合は、gn10の `pcd_to_esdf_converter` で事前に `.esdf` へ変換する。追跡ノードはPCLや自己位置探索を実行しない。
- `map_frame` と `camera_parent_frame` のTFを画像のstampで取得する。最新の位置を代用しない。待ち時間上限は `map_tf_wait_sec`、ポーリング周期は `map_tf_poll_sec`。Unityの時計が停止しても壁時計で待ちを打ち切る。
- `map_max_image_age_sec` より古い画像、対応するTFがない画像は処理しない。短いTF遅延では飛翔状態を保持し、`map_pose_loss_reset_sec` を超える途絶ではリセットする。
- JSONから距離場を作る範囲・解像度・最大メモリは `map_min/max_* / map_resolution_m / map_max_memory_mb`。`.esdf` 読み込み時はファイル内の範囲と解像度を使用する。
- `map_exclusion_distance_m` 内の静止構造物を除外する。地図外は `map_reject_unknown` で扱いを指定する。
- 床平面の除去は `map_use_floor / map_floor_z_m`。実測で飛翔を確認した後は、予測位置から `map_tracking_keep_radius_m` 内を一律の地図除去から保護する。大きさ・速度・予測ゲートの検証は継続する。
- 地図モードのプロファイルは `observed_landing_only: true`。低い点を実測した場合だけLANDED、未検出終了はLOST、空中静止はRESTING。従来プロファイルはfalseで既存の判定を維持する。
- 地図モードでは従来の実験室の壁を避けるための前方5m制限を6mへ広げている。実測更新のない画像で古い雑巾TFを新しい時刻に更新しない。カラー・デバッグ点群は比較用プロファイルでOFF。
- 設定は起動時に読み込む。方式や地図を切り替えるときはYAMLを選び直し、追跡ノードを再起動する。

### 状態確認

```bash
ros2 topic echo /cloth/ready
ros2 topic echo /cloth/background_status
ros2 topic echo /cloth/tracking_state
```

`MAP_READY` は時刻付き自己位置が取得できている状態。`MAP_WAITING_FOR_POSE` は自己位置待ち、`MAP_IMAGE_TIME_INVALID` は画像とROS時計の不整合、`MAP_PROCESSING_ERROR` はCUDA異常。`/cloth/ready` は背景準備済み・追跡中ではないことを通知するが、射出を強制的に止める機能ではない。

`cloth_test_11_camera_only` には対応地図・自己位置がないため、通常は従来モードで検証する。大会の地図に架空の自己位置を付けて地図方式の成功とは扱わない。

### 検証

`-DBUILD_TESTING=ON` でGPUの地図照合とCPUの着地判定テストを追加する。
`test/test_map_pose_guard.py` は独立したROS_DOMAIN_IDで、TF欠損、停止した時計、古い画像、自己位置復帰を検証する。
`test/verify_map_unity.py` はUnityと実際のgn10自己位置推定を起動した上で連投・走行停止後射出を検証する。
結果の `passed` は構成の機能検証、`all_landings_observed` は全投で着地高さまで実測できたかを表す。両者を混同しない。
地図方式の `--require-observed-landing` は全投の実測着地を必須にする品質確認用。

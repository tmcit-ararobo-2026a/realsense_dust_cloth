# realsense_dust_cloth

Intel RealSense D435 の **Depth（深度）情報のみ** を用いた、飛翔中の雑巾（布）の 3次元弾道リアルタイム検出・TF配信パッケージです（TMCIT 荒川ロボコン 2026A 向け）。

機体搭載の **NVIDIA Jetson** に RealSense を接続して超軽量に深度計測とTF配信を行い、手元の **操作用PC** で RViz2 監視や rosbag 記録を行う分散構成を標準としています。

---

## 1. ハードウェア構成・接続

### 1.1 USB 接続
* **接続先**: NVIDIA Jetson 本体の **USB 3.0 ポート（青色端子 / SSマーク）**
* **注意**: RealSense D435 は 60fps の深度ストリームを送信するため、必ず USB 3.0 ポートに直結してください（USB 2.0 ポートや非給電ハブでは帯域不足・認識エラーの原因になります）。

### 1.2 カメラ設置方法（ベルト先端・縦置き90°）
* **設置場所**: ベルト射出機構の直動先端部
* **設置向き**: **90° ロール回転させた縦置き**
  * カメラを縦向きにすることで、垂直視野角を約 90° に拡大（水平視野角は約 60°）。
  * 高く打ち上がる雑巾の最高到達点（頂点）を見失うことなく追尾します。
* **仰角**: 水平から **上向き 45°**

---

## 2. 特徴
* **色・照明非依存（No RGB / Depth Only）**:
  * カラー画像（RGB）を一切使わず、深度背景差分のみで動体を抽出。
  * 会場の照明変化、影、布の色や柄に全く左右されません。
* **相手ロボット誤認防止 ＆ 2種類の雑巾対応**:
  * 3D バウンディングボックス上限判定（$0.65\,\text{m}$）により、**通常雑巾（約48g / $200\times300\,\text{mm}$）** と **スーパー雑巾（4枚連結 約140g / 約$400\times600\,\text{mm}$）** を両方通過させ、巨大な **相手ロボット（$0.7\,\text{m}$超）** を瞬時に除外。
* **Jetson 負荷を考慮した超軽量設計**:
  * 深度差分 ＋ 空間マスク ＋ 重心抽出に特化。60fps でも Jetson の CPU をほとんど消費しません。
  * GUI 描画（RViz2）やログ保存は手元 PC に任せる分散運用が可能。

---

## 3. セットアップ（Jetson 側）

### 3.1 ドライバインストール
```bash
sudo apt update
sudo apt install -y ros-humble-realsense2-camera
```

### 3.2 ワークスペースのビルド
```bash
cd ~/ros2_ws
colcon build --packages-select realsense_dust_cloth
source install/setup.bash
```

---

## 4. 実験手順（Jetson + 操作用PC）

### ステップ 1: [Jetson 側] RealSense カメラの起動
Depth のみ 60fps（848x480）で起動します：
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.profile:=848x480x60 \
  enable_color:=false
```

### ステップ 2: [Jetson 側] 弾道トラッカーの起動（GUIなし）
Jetson 側では画面描画を省き、トラッカーと TF 配信のみを軽量実行します：
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch realsense_dust_cloth trajectory_recorder.launch.py rviz:=false
```
> [!NOTE]
> 起動直後の約 1 秒間はカメラの前に手や雑巾をかざさず静止させてください。背景を自動学習し、`Background calibrated! Ready to broadcast flying_cloth TF.` と出たら準備完了です。

### ステップ 3: [Jetson 側] rosbag で放物線を丸ごと記録
有線直結のローカル環境（Jetson）で直接保存するため、Wi-Fi のパケットロスがなく確実に記録できます：
```bash
source /opt/ros/humble/setup.bash
ros2 bag record /tf /tf_static /cloth/trajectory_marker -o cloth_experiment_bag
```
* 布を射出すると、布の飛翔に合わせて **`flying_cloth` TF が空間内を放物線を描いて移動** し、Jetson のストレージに記録されます。
* 射出実験が終わったら `Ctrl + C` で記録を停止します。

### ステップ 4: [手元 PC 側] RViz2 で 3D 弾道のリアルタイム監視（任意）
同一 Wi-Fi LAN に接続した手元のノート PC 等で RViz2 を起動し、目視確認します：
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/realsense_dust_cloth/rviz/cloth_tracker.rviz
```
* **黄色ライン（Flight Path）**: 実際の雑巾が描いた 3D 飛翔放物線（約3秒間残像表示）
* **赤色ポイント**: 雑巾の瞬時 3D 位置
* **緑色ライン**: `shooter_calculator` の計算による目標弾道（同時起動時）
* **視点操作**: 左ドラッグで 3D 自由回転、中ドラッグでパン、ホイールで拡大縮小が可能。

---

## 5. shooter_calculator との連携
実験で得られた各出力レベル（1〜10）ごとの初速・飛距離データは、付属の `trajectory_analyzer` で解析できます：
```bash
ros2 run realsense_dust_cloth trajectory_analyzer --data-dir <データ保存先>
```
出力されたルックアップテーブル（LUT）コードを `shooter_calculator_node.cpp` に反映することで、実際の布の空気抵抗やスリップを織り込んだ高精度自動照準が可能になります。

---

## 6. パラメータ設定 (`config/tracker_params.yaml`)

| パラメータ | 既定値 | 説明 |
| :--- | :---: | :--- |
| `depth_topic` | `/camera/camera/depth/image_rect_raw` | D435 深度画像トピック名 |
| `camera_info_topic` | `/camera/camera/depth/camera_info` | D435 カメラパラメータトピック名 |
| `min_depth_m` | `0.25` [m] | ベルト先端・機体フレームの至近距離除外マスク |
| `max_depth_m` | `6.0` [m] | 最大計測距離 |
| `diff_threshold_m` | `0.12` [m] | 背景より手前何mを動体とみなすかの閾値 |
| `max_cloth_size_m` | `0.65` [m] | 雑巾の最大3Dサイズ（スーパー雑巾を許容し、相手ロボットを除外） |
| `trajectory_history_sec` | `3.0` [s] | RViz2 上に放物線アークを残像表示する秒数 |

# realsense_dust_cloth

Intel RealSense D435 の **PointCloud2（3D点群）** を用いた、飛翔中の雑巾（布）の 3次元弾道リアルタイム検出・TF配信パッケージです（TMCIT 荒川ロボコン 2026A 向け）。

機体搭載の **NVIDIA Jetson** に RealSense を接続して点群クラスタリングとTF配信を行い、手元の **操作用PC** で RViz2 監視や rosbag 記録を行う分散構成を標準としています。

---

## 1. ハードウェア構成・接続

### 1.1 USB 接続
* **接続先**: NVIDIA Jetson 本体の **USB 3.0 ポート（青色端子 / SSマーク）**
* **注意**: RealSense D435 は 60fps の 3D点群ストリームを高速転送するため、**必ず USB 3.0 ポートに直結** してください。

### 1.2 カメラ設置方法（ベルト先端・縦置き90°）
* **設置場所**: ベルト射出機構の直動先端部
* **設置向き**: **90° ロール回転させた縦置き**
  * カメラを縦向きにすることで、垂直視野角を約 90° に拡大。
  * 高く打ち上がる雑巾の最高到達点（頂点）を見失うことなく追尾します。
* **仰角**: 水平から **上向き 45°**

---

## 2. 特徴（PointCloud 方式）
* **3D PassThrough 空間切り出し**:
  * ロボット機体枠（$X < 0.3\,\text{m}$）や床面（$Z < 0.05\,\text{m}$）を幾何学的にスパッとカット。
* **ボクセルダウンサンプリング & 高速 DBSCAN クラスタリング**:
  * 3cm ボクセル格子で間引きを行い、Jetson の CPU 負荷を最小限に抑えつつ 60fps 追尾。
* **相手ロボット誤認防止 ＆ 2種類の雑巾対応**:
  * 3D 直方体サイズ上限判定（$0.65\,\text{m}$）により、**通常雑巾（約48g / $200\times300\,\text{mm}$）** と **スーパー雑巾（4枚連結 約140g / 約$400\times600\,\text{mm}$）** を両方通過させ、巨大な **相手ロボット（$0.7\,\text{m}$超）** を瞬時に除外。
* **雑巾自身の 3D 点群可視化**:
  * 雑巾を構成する点群（`/cloth/cluster_points`）が RViz 上にエメラルドグリーンで描画され、空中で雑巾が開いているか丸まっているかを 3D で直感的に確認可能。

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

### ステップ 1: [Jetson 側] RealSense カメラの起動（PointCloud 出力）
`pointcloud.enable:=true` を指定して起動します：
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.profile:=848x480x60 \
  enable_color:=false \
  pointcloud.enable:=true
```

### ステップ 2: [Jetson 側] 弾道トラッカーの起動（GUIなし・PointCloudモード）
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch realsense_dust_cloth trajectory_recorder.launch.py rviz:=false
```

### ステップ 3: [Jetson 側] rosbag で放物線を丸ごと記録
有線直結のローカル環境（Jetson）で直接保存するため、Wi-Fi のパケットロスがなく確実に記録できます：
```bash
source /opt/ros/humble/setup.bash
ros2 bag record /tf /tf_static /cloth/trajectory_marker /cloth/cluster_points -o cloth_experiment_bag
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
* **エメラルドグリーン点群（Segmented Cloth）**: 飛翔中の雑巾クラスタ点群
* **赤色ポイント**: 雑巾の瞬時重心位置
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
| `points_topic` | `/camera/camera/depth/color/points` | RealSense PointCloud2 トピック名 |
| `min_x_m` / `max_x_m` | `0.30` / `6.0` [m] | 前後方向の抽出空間（ベルト先端手前を除外） |
| `min_y_m` / `max_y_m` | `-1.5` / `1.5` [m] | 左右方向の抽出空間 |
| `min_z_m` / `max_z_m` | `0.05` / `3.5` [m] | 高さ方向の抽出空間（床面・天井を除外） |
| `voxel_size_m` | `0.03` [m] | ボクセルダウンサンプリング解像度（Jetson負荷低減） |
| `cluster_eps_m` | `0.15` [m] | DBSCAN クラスタリング結合距離 |
| `max_cloth_size_m` | `0.65` [m] | 雑巾の最大3Dサイズ（スーパー雑巾を許容し、相手ロボットを除外） |

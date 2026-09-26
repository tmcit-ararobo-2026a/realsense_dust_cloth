# realsense_dust_cloth

Intel RealSense D435 と NVIDIA Jetson (CUDA) を C++ で密結合した、飛翔中の雑巾（布）の 3次元弾道リアルタイム追尾・TF配信パッケージです（TMCIT 荒川ロボコン 2026A 向け）。

## 1. 特徴
* **C++ & CUDA 密結合（60fps / 超低遅延 < 3ms）**:
  * RealSense C++ API (`librealsense2`) から取得した Depth データを直接 GPU メモリへ転送。
  * 3D 逆投影・姿勢変換・空間切り出し・Voxel間引き・クラスタリング・重心算出を **すべて GPU 内で完結** させ、Jetson の CPU 負荷を最小化。
* **rosbag 容量の激減（99.9% 削減）**:
  * 生の巨大な点群（毎分数十GB）を ROS 通信に流さず、算出された **雑巾の重心（`/tf`, `/cloth/position`）と軌跡マーカーのみ** を配信・記録。1分間でも **数百KB** に収まります。
* **相手ロボット誤認防止 ＆ 2種類の雑巾対応**:
  * 3D バウンディングボックス上限（$0.65\,\text{m}$）により、**通常雑巾** と **スーパー雑巾** を両方通過させ、巨大な **相手ロボット（$0.7\,\text{m}$超）** を瞬時に除外。

---

## 2. ハードウェア設置・接続

* **USB 接続**: Jetson 本体の **USB 3.0 ポート（青色端子 / SSマーク）に直結**（※USB 2.0 だと帯域不足で 30fps に制限されます）。
* **カメラ設置向き**: **90° ロール回転の縦置き**（垂直視野角を約 90° に拡大し、最高到達点を見失わない配置）。
* **取り付け仰角**: 水平から **上向き 20°** (`camera_pitch_deg: 20.0`)。
* **設置位置**: ベルト射出機構の直動先端部（機体中心から前方 $0.25\,\text{m}$、高さ $0.35\,\text{m}$）。

---

## 3. ビルド手順

```bash
cd ~/ros2_ws
colcon build --packages-select realsense_dust_cloth
source install/setup.bash
```
> **Tip**: 特定の GPU アーキテクチャに絞って超高速ビルドしたい場合（例: RTX 3050 は `86`、Jetson Orin は `87`、Xavier は `72`）:
> `colcon build --packages-select realsense_dust_cloth --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86`

---

## 4. 実行手順

### 4.1 [Jetson 実機] 射出実験の実行（GUIなし・最高速）

#### ステップ 1: CUDA トラッカー起動（RealSense 直叩きモード）
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch realsense_dust_cloth cloth_tracker_cuda.launch.py use_direct_realsense:=true
```
※ GUI（RViz2）は立ち上がらない完全なヘッドレス動作のため、Jetson のリソースを消費しません。

#### ステップ 2: rosbag で弾道を軽量記録
```bash
ros2 bag record /tf /cloth/position /cloth/trajectory_marker -o cloth_flight_bag
```
* 布が射出されると、**`flying_cloth` TF が空間内を放物線を描いて移動** し、軽量に記録されます。

---

### 4.2 [手元 PC] 3D 弾道の遠隔リアルタイム監視（任意）
同一 Wi-Fi LAN に接続した手元のノート PC 等で RViz2 を起動すると、Jetson に負荷をかけずに遠隔で弾道を目視確認できます：
```bash
rviz2 -d ~/ros2_ws/src/realsense_dust_cloth/rviz/cloth_tracker.rviz
```

---

### 4.3 [開発 PC] シミュレータによる 3D 可視化デモ（カメラ不要）
手元 PC だけで動作確認したい場合、物理運動シミュレータと RViz2 がワンタッチで起動するデモを用意しています：
```bash
ros2 launch realsense_dust_cloth demo_flight_rviz.launch.py
```
* エメラルドグリーンの点群（雑巾）が 2.5 秒ごとに放物線を描いて打ち上がり、黄色い軌跡ラインが描画されます。

---

## 5. 主要パラメータ (`config/tracker_params.yaml`)

| パラメータ | 既定値 | 説明 |
| :--- | :---: | :--- |
| `use_direct_realsense` | `false` | RealSense C++ API 直接ストリーミング有効化（Jetson実機時は `true` 推奨） |
| `camera_pitch_deg` | `20.0` [deg] | カメラ仰角（上向き 20°） |
| `camera_roll_deg` | `90.0` [deg] | カメラ縦置き角度（90° ロール） |
| `min_x_m` / `max_x_m` | `0.30` / `6.0` [m] | 前後方向の抽出空間（ベルト先端手前を除外） |
| `min_z_m` / `max_z_m` | `0.05` / `3.5` [m] | 高さ方向の抽出空間（床面・天井を除外） |
| `voxel_size_m` | `0.03` [m] | 空間ハッシュ Voxel 解像度（3cm 間引き） |
| `cluster_eps_m` | `0.15` [m] | 近傍結合クラスタリング探索距離 |
| `max_cloth_size_m` | `0.65` [m] | 雑巾の最大3Dサイズ（スーパー雑巾を許容し、相手ロボットを除外） |
| `publish_debug_cloud` | `false` | RViz 確認用クラスタ点群（`/cloth/cluster_points`）のパブリッシュ有効化 |

# realsense_dust_cloth

Intel RealSense D435 と NVIDIA Jetson (CUDA) を C++ で密結合した、飛翔中の雑巾（布）の 3次元弾道リアルタイム追尾・TF配信・自動放物線解析パッケージです（TMCIT 荒川ロボコン 2026A 向け）。

---

## 1. 特徴
* **C++ & CUDA 完全GPUパイプライン（60fps / 超低遅延 < 2ms）**:
  * 深度画像の 3D 逆投影・空間ROI・背景差分・Voxel間引き・クラスタリング・重心算出を **すべて GPU 内で一貫処理**。
  * Jetson の CPU 負荷はほぼゼロ（数%以下）で動作します。
* **生の実測点のみを忠実にプロット**:
  * 架空の補間点は一切生成せず、カメラが捉えた本物のフレーム位置（TF / 赤い点列）を出力。
  * クリティカルダンピング平滑化により、布のヒラヒラ展開による重心ノイズを吸収して滑らかな放物線を描画。
* **自動放物線ロギング & 最確軌道解析（重複自動排除）**:
  * 雑巾を飛ばすたびに自動で検知して CSV に記録。
  * 同じデータの重複保存を自動でスキップし、実験を重ねることで「最も確率の高い放物線 $Z(X)$」と「着地点のばらつき（$\pm 1\sigma$）」を全自動で算出。
* **RealSense C++ API (`librealsense2`) 直結モード内蔵**:
  * ROS 2 のカメラノードを別で立ち上げなくても、ノード単体でカメラから直接ゼロコピーで GPU にストリーミング可能。

---

## 2. ハードウェア設置

* **USB 接続**: Jetson の **USB 3.0 ポート（青色端子 / SSマーク）に直結**（※USB 2.0 だと帯域不足になります）。
* **カメラ設置向き**: **90° ロール回転の縦置き**（垂直視野角を約 90° に拡大し、放物線の最高到達点を見失わない配置）。
* **取り付け仰角**: 水平から **上向き 20°** (`camera_pitch_deg: 20.0`)。
* **機体オフセット**: 前方 $0.15\,\text{m}$、高さ $0.35\,\text{m}$。

---

## 3. Jetson でのセットアップ & 起動手順

### ステップ 1: Jetson に RealSense SDK をインストール
```bash
sudo apt update
sudo apt install -y librealsense2-dev librealsense2-utils

# カメラ認識確認（接続して実行）
rs-enumerate-devices
```

### ステップ 2: ワークスペースのビルド
```bash
cd ~/ros2_ws
colcon build --packages-select realsense_dust_cloth
source install/setup.bash
```
> **Tip**: CMake が自動で `librealsense2` を検知し、`#include <librealsense2/rs.hpp>` が有効化されます。

### ステップ 3: Jetson のパフォーマンス最大化
```bash
sudo nvpmodel -m 0   # パワーモード最大
sudo jetson_clocks   # GPU/CPU クロック最大固定
```

### ステップ 4: 起動（カメラ直結モード）
```bash
ros2 launch realsense_dust_cloth cloth_tracker_cuda.launch.py use_direct_realsense:=true
```
* 画面に `RealSense Hardware Pipeline streaming: 848x480 @ 60fps` と出れば成功です！

---

## 4. 放物線データの記録・解析方法

何回も発射実験を行い、データを記録して最確軌道を出したい場合は、別ターミナルで記録ツールを動かします：

```bash
python3 ~/ros2_ws/src/realsense_dust_cloth/realsense_dust_cloth/record_trajectories.py
```

* **自動記録**: 雑巾が飛ぶたびに自動で検知し、`data/trajectories_日時.csv` に実測座標が追記されます。
* **重複自動排除**: 同じ飛び方や同一 bag の再生物は自動でスキップされるため、データが汚れません。
* **終了とグラフ生成**: `Ctrl + C` を押すと記録が終了し、最確放物線 $Z(X)$ と着地点ばらつき（$\pm 1\sigma$）のグラフ画像（`.png`）が自動保存されます。

---

## 5. PCでの遠隔 RViz 可視化 / bag 再生

### 手元 PC から RViz2 を起動してリアルタイム目視
```bash
rviz2 -d ~/ros2_ws/src/realsense_dust_cloth/rviz/cloth_tracker.rviz
```

### 記録済み rosbag を再生してテスト
```bash
# ループ再生（何回も繰り返す場合）
ros2 bag play ~/ros2_ws/data/test_bag --loop
```

### rqt_bag で GUI 操作（シークバー・巻き戻し再生）
```bash
ros2 run rqt_bag rqt_bag &
```

---

## 6. 主要パラメータ (`config/tracker_params.yaml`)

| パラメータ | 既定値 | 説明 |
| :--- | :---: | :--- |
| `use_direct_realsense` | `false` | RealSense C++ API 直接ストリーミング（Jetson 実機時は `true` 推奨） |
| `camera_pitch_deg` | `20.0` [deg] | カメラ仰角（上向き 20°） |
| `camera_roll_deg` | `90.0` [deg] | カメラ縦置き角度（90° ロール） |
| `min_x_m` / `max_x_m` | `0.40` / `3.80` [m] | 前後方向の ROI 抽出空間 |
| `min_z_m` / `max_z_m` | `0.20` / `2.20` [m] | 高さ方向の抽出空間（床面・天井を除外） |
| `bg_diff_thresh_m` | `0.15` [m] | 背景差分閾値（静止した壁・机を消去） |
| `voxel_size_m` | `0.03` [m] | 空間ハッシュ Voxel 解像度（3cm 間引き） |
| `cluster_eps_m` | `0.15` [m] | クラスタ探索距離 |
| `max_cloth_size_m` | `1.50` [m] | 雑巾の最大サイズ（展開した布を包含） |

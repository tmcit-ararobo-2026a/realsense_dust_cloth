# realsense_dust_cloth

Intel RealSense D435 の **Depth（深度）情報のみ** を用いた、飛翔中の雑巾（布）の 3次元弾道リアルタイム検出・TF配信パッケージです（TMCIT 荒川ロボコン 2026A 向け）。

検出した布の 3次元位置を **TF（`base_link -> flying_cloth`）** としてリアルタイム配信し、**rosbag で記録するだけで放物線データを丸ごと保存** できます。

---

## 1. 特徴
* **色・照明非依存（No RGB / Depth Only）**:
  * カラー画像（RGB）を一切使わず、深度背景差分のみで動体を抽出。
  * 会場の照明変化、影、布の色や柄に全く左右されません。
* **ベルト直動先端・縦向き設置対応（上下 90° FOV）**:
  * D435 を 90° ロール回転させて縦置き（上下視野角 約90° / 横 約60°）。
  * 高く打ち上がる放物線の最高到達点（頂点）から着地までを画面外に見切れることなく追尾。
* **相手ロボット誤認防止 ＆ 2種類の雑巾対応**:
  * 超軽量な 3D バウンディングボックス上限判定（$0.65\,\text{m}$）により、**通常雑巾（約48g / $200\times300\,\text{mm}$）** と **スーパー雑巾（4枚連結 約140g / 約$400\times600\,\text{mm}$）** を両方通過させ、巨大な **相手ロボット（$0.7\,\text{m}$超）** を瞬時に除外・スキップ。
* **超軽量設計（Low Latency）**:
  * 重いファイル保存や物理計算を全廃し、深度差分 ＋ 空間マスク ＋ 重心抽出に特化。60〜90fps の高フレームレートでも極小の CPU 負荷で動作。
* **rosbag で丸ごと記録**:
  * `ros2 bag record /tf /tf_static` を実行するだけで、実験時の全放物線が TF として自動保存。

---

## 2. ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select realsense_dust_cloth
source install/setup.bash
```

---

## 3. 実機での動かし方

### ステップ 1: RealSense D435 カメラの起動
カラーカメラを停止し、Depth のみ 60fps（848x480）で起動します：
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.profile:=848x480x60 \
  enable_color:=false
```

### ステップ 2: 弾道トラッカー ＆ RViz2 の起動
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch realsense_dust_cloth trajectory_recorder.launch.py
```
* **起動後 1 秒間**: ロボットを静止させておくと、前方の空間深度を自動学習し、`Background calibrated! Ready to broadcast flying_cloth TF.` と表示されて準備完了になります。
* **RViz2 が自動起動** し、ロボット座標（`base_link`）とカメラ姿勢が表示されます。
* ※ RViz2 を起動したくない場合は `rviz:=false` を指定してください。

### ステップ 3: rosbag で放物線を記録開始
```bash
source /opt/ros/humble/setup.bash
ros2 bag record /tf /tf_static -o cloth_experiment_bag
```
* 布を射出すると、布の飛翔に合わせて **`flying_cloth` TF が空間内を放物線を描いて移動** し、rosbag に記録されます。
* 射出実験が終わったら `Ctrl + C` で記録を停止します。

---

## 4. 記録した放物線の再生・確認

記録したバッグファイルを再生して、RViz2 で実際の放物線を目視確認できます：

```bash
# ターミナル 1: バッグを再生
ros2 bag play cloth_experiment_bag

# ターミナル 2: RViz を起動
rviz2
```
* RViz2 で **Fixed Frame** を `base_link` にし、**TF** を表示すると、射出された布の放物線がそのまま空間内に再生されます。

---

## 5. パラメータ設定 (`config/tracker_params.yaml`)

| パラメータ | 既定値 | 説明 |
| :--- | :---: | :--- |
| `depth_topic` | `/camera/camera/depth/image_rect_raw` | D435 深度画像トピック名 |
| `camera_info_topic` | `/camera/camera/depth/camera_info` | D435 カメラパラメータトピック名 |
| `min_depth_m` | `0.25` [m] | ベルト先端・機体フレームの至近距離除外マスク |
| `max_depth_m` | `6.0` [m] | 最大計測距離 |
| `diff_threshold_m` | `0.12` [m] | 背景より手前何mを動体とみなすかの閾値 |
| `max_cloth_size_m` | `0.65` [m] | 雑巾の最大3Dサイズ（スーパー雑巾を許容し、相手ロボットを即座に除外） |

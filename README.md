# realsense_dust_cloth

RealSense D435 の **Depth（深度）情報のみ** を用いた、飛翔中の雑巾（布）の 3次元弾道計測および期待値放物線モデリングパッケージです（TMCIT 荒川ロボコン 2026A 向け）。

---

## 1. 特徴
* **色・照明非依存（No RGB）**: 照明変化や布の色・柄に影響されない、Depth背景差分による高速動体抽出。
* **ベルト直動先端・縦向き設置対応**: D435 を 90° 縦置き（上下視野角 90° / 横 60°）にすることで、高く打ち上がる放物線の頂点から着地までを見切れずに追尾。
* **自動レコーディング**: 射出の動き出しから着地までを自動で 1 試行として切り出し、CSV/JSON に保存。
* **期待値放物線モデル生成**: 複数回実験の弾道データから、空気抵抗付き物理モデルを用いて最も命中確率の高い中心放物線を算出。
* **shooter_calculator 連携**: 算出した期待値データを `shooter_calculator` の LUT に直接フィードバック可能。

---

## 2. ビルド
```bash
cd ~/ros2_ws
colcon build --packages-select realsense_dust_cloth
source install/setup.bash
```

---

## 3. 使い方

### ① 弾道計測ノードの起動（実験時）
ロボットのベルト直動先端に D435 を縦置きした状態で以下を実行します。
```bash
ros2 launch realsense_dust_cloth trajectory_recorder.launch.py output_level:=1
```
* `output_level:=1〜10`: 現在テストしている射出出力レベル（データラベル用）
* 静止状態から布が射出されると、自動で追跡・計測を行い、`/tmp/cloth_trajectories/` に試行ごとの JSON ログが保存されます。

### ② 期待値データの解析・LUT生成
同一出力で複数回射出したあと、解析スクリプトを実行します。
```bash
ros2 run realsense_dust_cloth trajectory_analyzer
```
* 各出力レベル（1〜10）ごとの平均初速（Release Velocity）や抗力係数が計算され、`shooter_calculator` にそのまま貼り付け可能なコード形式で出力されます。

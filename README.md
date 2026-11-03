# realsense_dust_cloth

RealSenseで飛翔中の雑巾を追跡するROS 2 / CUDAパッケージです。地図と自己位置の管理・地図による背景除去は `gn10-pointcloud-localization` が担当します。追跡側の独立地図読込、深度背景学習、自己位置を使わない背景差分方式は削除しました。

## 処理の流れ

```text
RealSense深度画像 → 3D化・ROI・Voxel間引き
 → /realsense/cropped_cloud
 → gn10：自己位置とESDF地図で静的背景を除去
 → /realsense/dynamic_cloud
 → 候補クラスタ選択・射出確認・予測追跡
```

追跡確定後の予測保護球は `/cloth/filter_prediction` に送ります。RealSenseの点群を自己位置推定の計算には使いません。地図差分後にも地図にない静止物は残るため、大きさ・移動量・速度・予測との整合で候補を選びます。

## 設定は3ファイル

| YAML | 用途 | 時計 | RealSense直結 | 初期入力 |
|---|---|---|---|---|
| `config/tracker_params.yaml` | Jetson実機 | 実時間 | true | 深度 |
| `config/unity_params.yaml` | Unity | `/clock` | false | 画像トピック |
| `config/rosbag_params.yaml` | rosbag | `/clock` | false | 記録済み差分点群 |

数値・取付位置・ROI・追跡条件は選んだYAMLが正本です。必須設定が欠けていると起動時にエラーになります。launchは指定YAMLを別ファイルへ差し替えたり、時刻・カメラ設定を黙って上書きしたりしません。YAMLの変更後は追跡ノードを再起動してください。

## 起動

```bash
source ~/ros2_ws/install/setup.bash
# 実機：通常のMID360・IMU・localizationを別途起動しておく
ros2 launch realsense_dust_cloth cloth_tracker.launch.py
# Unity：localizationもシミュレーション時刻で起動しておく
ros2 launch realsense_dust_cloth cloth_tracker.launch.py cloth_tracker_params_file:=unity_params.yaml
# rosbag
ros2 launch realsense_dust_cloth cloth_tracker.launch.py cloth_tracker_params_file:=rosbag_params.yaml
```

※ 引数 `cloth_tracker_params_file`（後方互換として `params_file` も可）で設定YAMLを指定します。

`use_sim_time:=true`、`use_direct_realsense:=false`、`sim:=true` の追加指定は不要です。設定したい値はYAML内で変更します。

Jetsonは追跡とlocalizationを実行し、手元のPCでRVizを開けます。PCとJetsonのROS通信設定を合わせてください。

```bash
rviz2 -d ~/ros2_ws/src/realsense_dust_cloth/rviz/cloth_tracker.rviz
```

同じPCでRVizも起動したい場合はYAMLの `start_rviz`、CSV解析は `start_analyzer`、入力記録は `record_inputs` をtrueにします。補助ノードの時計も同じYAMLに従います。

直接接続にはlibrealsense2対応のビルドが必要です。直結指定でSDKがない場合、画像購読へ黙って切り替えずエラーにします。同じカメラを複数ノードから開かないでください。

## rosbagの入力段階

`rosbag_params.yaml` の `input_source` で、収録データのどの段階から再検証するか指定します。追跡方式はいずれもlocalization連携方式です。

- `dynamic_cloud`：記録済み `/realsense/dynamic_cloud` を再追跡。画像・カメラ情報は不要。上流で除去された点は復元できず、背景フィルタの再評価にはなりません。
- `cropped_cloud`：記録済みの除去前点群をgn10へ再投入。元のクロップ点群を `/bag/realsense/cropped_cloud` へremapし、入出力ループを避けます。gn10側の自己位置・地図に必要な入力も揃っている必要があります。
- `depth`：深度画像・camera_infoから全経路を再計算。自己位置推定用センサ、地図、カメラ取付設定も必要です。

差分後点群だけを再追跡する例：

```bash
ros2 bag play /path/to/bag --clock 100 --topics /realsense/dynamic_cloud
```

記録済み `/cloth/position`、着地・状態・マーカーは、新しい出力と混ざるため同じトピックで再生しません。全経路の再計算では、記録済みクロップ点群・差分点群・予測マーカーも入力から除きます。bagで収録したmap TFと、新たなlocalizationのTFを二重配信しないよう、再生する入力を選びます。

カメラの固定TFをbagから再生する場合は `publish_static_tf: false` にします。深度画像を再計算するときは、YAMLの取付位置・角度が記録時と一致していることを確認してください。

## 調整と原因の切り分け

YAMLは実行環境、カメラ、ROI、射出、飛翔、GPU、gn10接続、表示・ログの項目に分けています。

| 状態・症状 | 確認する内容 |
|---|---|
| `RESULT_TIMEOUT` | 差分後点群の配信、gn10の状態、時計、入力データの有無 |
| `IMAGE_TIME_INVALID` | 実機/Unity/bagの時計と入力の遅延 |
| `FILTER_*` | gn10の `realsense:` 設定、自己位置、TF、地図 |
| `IDLE` のまま | 入力点が残っているか、ROI、射出枠、最小点数・移動量 |
| 飛翔中に `LOST` | 欠測、物理条件、上流での除去、点数・予測とのずれ |
| 雑巾が上流で消える | gn10の高さ上限、床閾値、地図距離、最小動的点数 |

特に、gn10の `realsense.height_max` が射出位置や軌道より低いと、追跡側に届く前に消えます。追跡側の点数しきい値を下げても、上流で消えた点は戻りません。

`publish_debug_cloud: true` でROI枠、探索後の点群、候補クラスタを表示できます。GPUからの転送と通信負荷が増えるので、調査後はfalseに戻して比較します。点群が空でも、デバッグ配信が無効なだけなのか、実際に点がないのかを区別してください。

## 状態と出力の意味

- `TRACKING`：実測候補を追跡中。
- `LANDED`：採用した実測点が `landing_min_z` 以下になった。
- `LOST`：異常候補、欠測、時間切れで終了。空中で切れた位置を着地位置として発行しない。
- `RESTING`：着地高さより上で静止した。着地成功として扱わない。
- `/cloth/position` と `/cloth/measured_path` は採用した実測値。欠測中の予測点を実測として混ぜない。
- `/cloth/landing_position` は実測によるLANDED時のみ。最後の位置・軌跡は表示に残る。
- `/cloth/ready` は準備状態の通知であり、射出装置を自動で禁止する信号ではない。

## コードの入口

| 役割 | ファイル |
|---|---|
| 起動・終了の順序 | `src/cloth_tracker_cuda_node.cpp` |
| YAML読込・設定検証 | `src/tracker_parameters.cpp` |
| 画像トピック入力・カメラTF | `src/camera_input.cpp` |
| USB直結のRealSense取得 | `src/realsense_camera.cpp` |
| 非同期の色画像配信 | `src/color_stream.cpp` |
| 実測位置・着地・軌跡の配信 | `src/tracking_output.cpp` |
| 点群の受信・gn10への送信・入力途絶処理 | `src/external_cloud_tracking.cpp` |
| ROI・間引き・クラスタ選択 | `src/cuda_kernels.cu` |
| 射出確認・予測・異常と着地の判定 | `src/ballistic_tracker.cpp` |
| 状態トピック | `src/tracking_status.cpp` |

今回の削除・変更・設定名の移行は `CHANGES_20261102.md` に記載します。Jetsonの負荷と、雑巾を最後まで追える割合は実機で測定する必要があります。

コードを読む順序・今回の整理・削除した処理は `CODE_STRUCTURE.md` にまとめています。

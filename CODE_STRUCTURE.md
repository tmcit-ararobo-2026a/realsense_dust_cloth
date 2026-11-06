> 2026-11-06: 現行の深度経路はGPUリングバッファ背景差分で直接追跡する。以下は初回整理時の記録。現在の構成・修正・負荷の注意点は [Jetson負荷確認](JETSON_LOAD_REVIEW_20261106.md) を参照。

# コードの読み方と今回の整理

## 処理を読む順序

1. `src/main.cpp` でROSノードを起動する。
2. `src/cloth_tracker_cuda_node.cpp` でYAML読込、配信設定、カメラの開始と終了を行う。
3. 深度入力の場合、`src/camera_input.cpp` が深度からROI点群を作り、gn10へ送る。
4. `src/external_cloud_tracking.cpp` がgn10の動的点群を受け、探索範囲と候補クラスタを選ぶ。
5. `src/ballistic_tracker.cpp` が射出を確認し、飛翔中の候補を予測と照合する。
6. `src/tracking_output.cpp` が採用した実測位置と状態変化を配信する。

| ファイル | 担当 |
|---|---|
| `src/main.cpp` | ROS 2のエントリーポイント |
| `src/cloth_tracker_cuda_node.cpp` | 起動・終了の順序 |
| `src/tracker_parameters.cpp` | YAMLの必須設定読込・起動時の値検証 |
| `src/camera_input.cpp` | 画像トピック購読、カメラTF、深度点群の作成 |
| `src/realsense_camera.cpp` | USB直結のRealSense取得・ハードウェア設定 |
| `src/color_stream.cpp` | 最新の色画像のデコード・回転・配信。深度とは別スレッド |
| `src/external_cloud_tracking.cpp` | gn10との入出力、タイムアウト、探索範囲と候補選択 |
| `src/ballistic_tracker.cpp` | 射出確認、飛翔予測、候補判定、着地・途絶・高所静止 |
| `src/tracking_output.cpp` | 実測履歴、着地通知、TF、軌跡とROI表示 |
| `src/tracking_status.cpp` | 入力準備・追跡状態の通知 |
| `src/cuda_kernels.cu` | GPUでの深度変換・ROI・Voxel間引き・クラスタ処理 |

`cloth_tracker_cuda_node.hpp` の関数宣言とメンバーも、この役割ごとに並べた。`publish_input_status` が配信するトピック名 `/cloth/background_status` は既存の接続を保つため維持している。

## 長い関数の整理

- 1000行以上あったノード実装を、上記の役割ごとのcppへ分割した。
- `BallisticTracker::update` は時刻を確認し、射出前か飛翔中かに応じて `update_launch_candidate` / `update_flight` を呼ぶ。
- 射出枠の重複した境界判定を `is_inside_launch_gate` にまとめた。
- 点群の再投入・フィルター状態通知・タイムアウトを、名前付きコールバックに分けた。
- 候補選択を `find_cloth_candidate`、状態変化を `publish_tracking_transition`、実測履歴を `remember_observation`、表示を `publish_tracking_visualization` に分けた。
- 同じ時間差を表す複数の変数や、採用後に到達しない判定、重複したコメントを整理した。
- `.clang-format` を追加して、C++/CUDAの改行・インデント・include順を統一した。

## 削除した処理

| 削除対象 | 理由 |
|---|---|
| `include/realsense_dust_cloth/parabola_fit.hpp` | フィッティング関数がどこからも呼ばれていなかった。実際の飛翔予測はBallisticTrackerが担当する |
| `realsense_dust_cloth/sim_cloth_flight.py` | 背景学習時代の簡易カメラ生成デモ。現在のlaunch・検証から使われず、自己位置と地図入力も生成しない。シミュレーション検証はUnityを使う |
| `launch_x / launch_y / launch_z` | 代入されるだけで読み出しがない状態変数 |
| ROI描画の旧方式切替引数 | 呼出元では常にtrueであり、方式切替が残っているように見えていた |
| C++ターゲットのcv_bridgeビルド依存 | C++側では使っていない。Python画像中継ツールに必要な実行時依存は保持 |

CSV・PNGなどの実験データと、現在使われるPythonツールは残した。

| Pythonツール | 用途 |
|---|---|
| `trajectory_probability_analyzer.py` | `start_analyzer: true` で起動する軌道記録・解析 |
| `record_trajectories.py` | 個別の軌道記録・CSV出力 |
| `decompress_image.py` | 圧縮画像だけを持つbag向けの画像表示中継 |

## 挙動の扱いと確認

追跡の条件、ROSトピック名、YAML項目、設定値、起動コマンドは変えていない。3つのYAMLは整理前とバイト単位で一致する。

唯一の表示文言変更として、未着地でも表示していた `Landing` ラベルを `Last observation X` に直した。マーカーの位置は変更していない。

- colconビルドと既存のCTest 3件を実行。
- ノイズ・欠測・外れ値を含む120試行×200フレームで、整理前後の追跡状態・位置・速度・終了理由を比較。
- `cloth_debug_20261030_163742` を同じ設定で再生し、従来の位置出力と状態遷移を比較。
- CUDAカーネルと既存テストの変更は整形のみで、include順以外のC++トークンが一致することを確認。

この整理は追跡精度の改善を主張する変更ではない。既知の途中LOSTやJetsonでの最新版の確認状況は `VERIFICATION_20261102.md` を参照する。

今回の実行結果は、CTest 3/3成功。24,000フレームの状態・位置・速度・理由の比較は完全一致。
実bagは整理前後とも1416フレーム受信・9点採用・`IDLE → TRACKING → LOST` で、位置差は最大約8.4×10^-7m。既知のLOSTを成功とは扱っていない。

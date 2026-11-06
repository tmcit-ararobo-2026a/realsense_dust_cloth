# GUI・README・RViz・負荷の確認（2026-11-06）

PR #8のブランチで作業。mainと実機の起動中ノードは変更していない。前回のUnity6投中2投の終盤ロストが残るため、今回もマージ保留。

## 変更一覧

| ファイル | 変更 |
|---|---|
| README.md | 現在の深度背景差分経路、3プロファイルの実際の入力、起動、時計、GUI復元、RViz、負荷と未解決点へ更新。旧「localization必須」の説明を訂正 |
| realsense_dust_cloth/param_tuner_gui.py | 固定コミット値へ戻す機能を削除し、セッション中の調整を戻す機能へ置換。ワイルドカードYAML読込・カメラRoll範囲・rosbag初期保存先を修正。ROS変更は一括反映でき、拒否時にはYAMLを保存しない |
| src/tracking_output.cpp | 視聴者がいないマーカー・Pathは組み立てない。追跡中も表示間隔を適用し、開始/終了は即時表示。位置・TFは維持。毎点INFOログをDEBUGへ |
| src/tracking_status.cpp | 同じ入力状態・ready・追跡状態の繰返し配信を抑制。Transient Localは維持 |
| src/tracker_parameters.cpp | 軌跡表示間隔の実行中変更と不正値拒否を追加 |
| rviz/cloth_tracker.rviz | 標準は位置と黄色い軌跡。点群・ROI・TF・重複Pathは必要時だけ有効化。視点の中心・距離を調整 |
| config/*.yaml | 既存のstatic_marker_interval_secが追跡中の表示にも使われることを日本語コメントへ反映。数値は変更しない |
| test/test_gui_session.py | Qtの実ウィジェット、一時YAML、実ROSパラメータサービスで変更・拒否・元値復元を検証 |
| test/test_runtime_pipeline.cpp | 状態の変化時配信、後から購読した状態の取得、軌跡配信頻度を追加検証 |
| CMakeLists.txt / package.xml | GUIテストとPyQt5実行依存を追加 |
| docs/*.png | 実際のGUI画面・RViz描画の確認画像 |

## GUIの復元仕様と確認

「調整前へ戻す」は、同じGUIセッションで変更したパラメータを変更前へ戻す。ノードとYAMLは元値を別々に保存し、環境ごとの値を混ぜない。YAMLの数値表記・コメントを保ち、GUIが触っていない項目を巻き戻さない。再読込しても元値は維持する。GUI終了後は復元記録を保持しない。

既存の「追跡&背景リセット」は別物で、パラメータを戻す機能ではない。PCのGUIはPC上のYAMLを保存する。Jetsonのファイルへ自動転送する機能はない。

GUIを実表示し、新しいボタン、選択プロファイル、camera_roll_deg=90.0が正しく表示されることを確認した。一時YAMLで確認し、実機YAMLは変更していない。

## RVizの確認

実際のrviz2起動とトピック受信を確認。その後、同じRViz標準ライブラリ・標準プラグイン・設定ファイルを使う確認用プログラムで、Unityの新しい射出を表示し、RenderWindowのcaptureScreenShotから描画を保存した。既存トラッカー出力bagを表示しただけではない。

最終設定のPosition表示は27件受信・Transform: Ok、Markerは342件受信。点群・ROI・TF・Pathは無効でも赤い位置と黄色い実測軌跡を描画できた。位置表示と軌跡表示の動作確認であり、追跡成功の確認とは分ける。追加の表示確認射出も終盤にLOSTした。生深度bagの再生確認でも、一方の投擲は途中LOSTしたため、bagの安定追跡が改善済みとは扱わない。

## 負荷の評価

不要なデバッグ抽出、表示組立・配信、状態の繰返し、毎点INFOログを減らした。位置・TFと入力の追跡周期は維持する。ROS Domain 96/localhost限定で検証し、実機に動作指令は送っていない。

最大限の最適化完了とは断定しない。全点対クラスタ結合O(N²)、全ROI処理、同期・メモリ転送は残る。JetsonのCPU/GPU使用率と他ノードとの競合は実測が必要。既存SSH設定先192.168.2.1への読取接続はタイムアウトし、Jetsonかどうかの機種確認も負荷計測もできなかった。

PCのCUDA Releaseビルド成功。CTest 6/6成功（gui_session / launch_profiles / point_cloud_io / observed_landing / tracking_recovery / runtime_pipeline）。USB直結部分はPCにlibrealsense2開発SDKがないため未ビルド。

ローカルの詳細記録: `/home/akeru/Documents/Codex/2026-10-26/new-chat/outputs/gui_rviz_load_20261106/`。

# C2の開き方・再生成

FreeCAD 1.1.4、同梱Python 3.11、NumPyを使用。外部CAD部品・未同梱の形状ファイルには依存しない。

## 開く

`rotor-neutral.FCStd` をFreeCADで開く。4状態のFCStdとSTEPは同じ設計版から生成する。`rotor-max-input` は複合入力と翼ごとに異なるflap/lag、`rotor-parked` はピン引抜き済み、`rotor-folded` は全長翼の収納状態。

## 編集

モデルツリーの `Parameters` でCollective、CyclicCos、CyclicSin、Azimuth、FlapAngles、LagAnglesを変更し、同梱 `Regenerate_Active.FCMacro` を実行する。新しい文書を生成するので元モデルは保持される。通常のFreeCAD再計算だけでは形状を再構成しない。主要寸法はDesignJSONに格納し、閉路解と実形状へ反映する。リンク長+2 mm、取付位置+5 mmの再生成を検査する。

折畳みはParkedを有効にし、Azimuth=0、FlapAngles/LagAngles=全て0としてFoldFractionを0〜1へ設定。`operating-state.md` の準備条件を満たした停止姿勢のみ受け付ける。

## コマンドによる再生成・検査

FreeCADの同梱Pythonを使い、このディレクトリを作業ディレクトリとして以下を実行する。`python` はFreeCAD/PartをimportできるPythonを指す。

```text
python src/build.py
python src/test_mechanism.py
python src/mechanism.py
python src/control_checks.py
python src/structure.py
python src/check_cad.py
python src/check_critical.py
```

`check_cad.py` は診断台帳を生成する。ログを読まずに終了コードだけで全設計合格とはしない。折畳み結果の `minimum_gap_mm=0` は、AABBが重なる候補の保守的下限を含み、真の最小BRep隙間の実測値ではない。交差体積検査と公差余裕は別に判断する。

画像はFreeCAD GUIで `src` をsys.pathへ追加し、`import render; render.render()` を実行する。`render.animation()` は有限リンク閉路から12姿勢を再構築し、画像を生成する。低速表示の動作図で、210 rpmの実時間や空力弾性のシミュレーションではない。生成動画は頭部のみ表示し、全長翼は保存CADに含まれる。

## 成果物と限界

`requirements.csv` が全28要求の受入判定、`final-report.md` が要約。設計値・荷重・運用はdesign-basis/design-structure/operating-stateを参照する。固定接合の未完了、軸受/エラストマー/複合材保持容量の未確定があるため、初期設計の完成品として使用する段階ではない。

`verification/manifest.sha256` に納品ファイルのSHA-256を記録する。変更したCAD・コードに対し検証を再実行する。旧C1の結果や旧C2途中版の検査を最終版の証拠に混ぜない。

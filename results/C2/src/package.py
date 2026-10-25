"""Assemble an evidence-bound delivery; does not promote open design gates."""
from pathlib import Path
import csv,hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[1]

def run():
    cad=json.loads((ROOT/'verification/cad-checks.json').read_text())
    critical=json.loads((ROOT/'verification/critical-cad-checks.json').read_text())
    controls=json.loads((ROOT/'verification/control-checks.json').read_text())
    numeric=json.loads((ROOT/'verification/numerical.json').read_text())
    for name,state in cad['states'].items():
        assert state['native']['all_parts_valid'] and state['step']['valid'],name
        assert not critical[name]['bad_pairs'],name
        assert critical[name]['relative_volume_error']<1e-6,name
    for v in cad['parameter_variants'].values():
        assert v.get('all_parts_valid') and v['top_vs_solver_mm']<1e-6
    b=cad['states']['rotor-folded']['native']['bounds_mm'];width=b[3]-b[2]
    assert width<=9300
    rows=[
      (1,'合格','design-basis.md','参照方式と独自寸法を区別','実機の複製ではない'),
      (2,'合格','design-basis.md; design-structure.md','事実・仮定・導出・未解決を単位付きで区分','材料400 MPaは独自スクリーニング値'),
      (3,'未検証','design-review.md','自由度・荷重経路・閾値を整合','主保持・軸受容量・境界荷重が未確定'),
      (4,'合格','design-basis.md; verification/manifest.sha256','C1との差分と再検証対象を記録','変更後は再実行する'),
      (5,'未検証','src/build.py; verification/numerical.json','5翼・72度・個別flap/lag/pitchを構築','弾性支持の物理容量と結合を確定していない'),
      (6,'未検証','src/mechanism.py; src/build.py; images/swash-section.png','RR-Sピン軸から固定/回転リング時計角を解く','実支持・軸受容量と全域干渉は未閉鎖'),
      (7,'合格','verification/numerical.json; verification/cad-checks.json','3入力rank3と実CAD球中心の有限長閉路','油圧源・空力操縦負荷は別途'),
      (8,'合格','design-basis.md; src/test_mechanism.py','座標、符号、位相反転の負の試験','空力応答位相を確定した意味ではない'),
      (9,'未検証','verification/numerical.json; verification/control-checks.json','全周・複合入力の数値格子と4状態CAD照合','連続域と全自由度組合せの完全検証ではない'),
      (10,'未検証','verification/control-checks.json; verification/critical-cad-checks.json','首振り・レバー・一定長ロッド・室内ピストン位置','20度限界は形状上の設定、製品定格と全支持端の確認は残る'),
      (11,'未検証','verification/control-checks.json; verification/structure.json','ダンパー実ストローク、作用線、正常/劣化の縮約感度','温度/速度特性・製品性能未確定'),
      (12,'未検証','operating-state.md; src/build.py','翼根ストップ・パーク支持・マスト固定を追加','衝撃/風/全低回転条件の保持容量未確定'),
      (13,'未検証','design-review.md; verification/control-checks.json','弾性操縦反力の符号付き釣合いと主要経路','全空力・過渡・境界の6成分荷重は未閉鎖'),
      (14,'未検証','verification/structure-load-cases.csv','停止、始動、ホバー、端入力、不確かさ、地上風を整理','複合ケース・過渡荷重包絡が未確定'),
      (15,'不合格','verification/structure.json; design-structure.md','φ109.8ピン、リンク、カフ等の初期金属評価','軸受・エラストマー・桁接合・固定接合の容量が未完了'),
      (16,'未検証','verification/structure.json; src/build.py','積層外形、剛性仮定、圧縮/せん断需要を分離','材料許容、接着、変形、温度、遠心保持を未検証'),
      (17,'合格','design-review.md','疲労、共振、空力弾性、摩耗等のリスクと次工程を特定','解析・試験自体の完了ではない'),
      (18,'未検証','operating-state.md; src/mechanism.py','段階順序と機械保持前提、負の状態試験','タイダウンとトラップドキー実機構未完了'),
      (19,'合格','verification/cad-checks.json','固定Y投影の全長5翼・全ヘッドの包絡測定','甲板・格納庫は独自境界であり実機寸法でない'),
      (20,'未検証','verification/folding-clearance.csv','ピン引抜き後の翼根/桁/全長翼と自翼金具を1度/0.5度で掃引','全翼相互・全ヘッド・機体・公差を閉じた証明ではない'),
      (21,'未検証','operating-state.md; verification/structure.json','飛行ピン、引抜きガイド、マスト位置決めを構築','収納保持・誤操作/故障時の物理インターロック未完了'),
      (22,'未検証','bom.csv; images/; verification/neutral-overlaps.csv','名前・材質区分・接続先、正規モデルと断面を分離','固定接合の重なりが残り製法/締結を確定していない'),
      (23,'合格','Regenerate_Active.FCMacro; verification/cad-checks.json','リンク+2 mmと取付+5 mmの実CAD再生成','通常の再計算のみでは再生成しない。主要寸法全ての頑健性保証ではない'),
      (24,'未検証','operating-state.md; images/joint-detail.png','分割ソケット/外輪、ピン・ガイド・組立手順','全接合の抜け止め、工具アクセス、支持ハウジング組立が未閉鎖'),
      (25,'未検証','verification/neutral-overlaps.csv; verification/critical-cad-checks.json','中立全対診断と4状態の重要な非接触対を確認','全作動域・公差・変形、固定接合の製法別処理が残る'),
      (26,'合格','verification/cad-checks.json; verification/critical-cad-checks.json','4状態の新規プロセス読込、形状・STEP照合','STEPは形状であり機構履歴を保持しない'),
      (27,'合格','images/review.md; images/','全景・断面・最大入力・収納・関節詳細を目視確認','画像は構造容量の証明ではない'),
      (28,'合格','requirements.csv; final-report.md','28要求を一対一で追跡し未達を明記','追跡表の存在と設計完了は別'),
    ]
    with (ROOT/'requirements.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['requirement_id','status','evidence','method','limitations'])
        w.writerows([f'R-{i:02d}',status,evidence,method,limit] for i,status,evidence,method,limit in rows)
    counts={s:sum(row[1]==s for row in rows) for s in ['合格','不合格','未検証']}
    max_step=max(v['relative_volume_error'] for v in critical.values())
    closure=max(s['pitch']['max_ball_closure_mm'] for s in cad['states'].values())
    report=f'''# C2 メジャーアップデート — 最終報告

**CADモデルと検証一式を作成したが、「実際の機体の初期設計にそのまま使えるレベル」という依頼の完了には未達。G0完了、G1〜G3未通過。** 目標を展示模型へ変更していない。

## 作成したもの

425形状フィーチャ/648ソリッドのネイティブCADとSTEPを、中立・複合最大入力・停止準備・収納の4状態で保存。C1の球と棒を、内外球面、フォーク、ピン、保持部を備える継手へ作り直した。ピン軸を持つ固定/回転RR-S、球面スライダと分割ソケット、溝付き内外輪、停止支持、マスト位置決め、ピン引抜きガイドを含む。形状フィーチャ数は製造部品点数とは同義でない。

動作画像は有限長閉路から12姿勢の実CADを再生成したもの。ピッチを直接正弦波で回した表示ではない。`src`、再生成マクロ、BOM、設計基準、運用手順を同梱した。

## 確認した結果

- 4状態とも有効ソリッド、STEP再読込有効。STEP相対体積差最大 {max_step:.3g}。
- 実CADの球中心から測定したピッチリンク閉路誤差最大 {closure:.3g} mm。
- 3入力写像rank=3。リンク長+2 mm、取付位置+5 mmで実形状を再生成し閉路を照合。
- 2,880組の5翼・実サーボ検査で、眼間距離約{controls['servo_eye_range_mm'][0]:.1f}〜{controls['servo_eye_range_mm'][1]:.1f} mm、ダンパー約{controls['damper_eye_range_mm'][0]:.1f}〜{controls['damper_eye_range_mm'][1]:.1f} mm、最大首振り約{controls['max_joint_misalignment_deg']:.2f}°。
- サーボ/ダンパーのロッドが一定長500/350 mmであることを形状体積から検査。
- `critical-cad-checks.json` に列挙した重要な非接触対は4状態で交差体積なし。中立の全対診断は別のCSVに残す。
- 自翼の折畳み金具に対する1°/0.5°掃引で正の交差体積なし。最小隙間欄の0はAABBによる保守下限を含み、全系隙間の合格証拠ではない。
- 収納幅 {width:.3f} mm ≤ 9300 mm。全長 {b[1]-b[0]:.1f} mm、高さ {b[5]-b[4]:.1f} mm。全長翼を使用し機体左右軸を固定。

## 受入判定と残課題

要求28項目: 合格{counts['合格']}、不合格{counts['不合格']}、未検証{counts['未検証']}。部分的な証拠がある項目も、全要求を閉じなければ未検証とした。

特に、軸受・エラストマー・翼桁接合の保持容量、固定接合の製法/締結、空力と過渡を含む荷重包絡、全作動域の公差/変形込み干渉、収納保持/物理インターロックが残る。金属の400 MPaスクリーニングを通ったことや、CADが動くことを、これらの代用にしない。

**改善版の機構CADとしては引き渡せるが、初期設計完成品、製作承認図、飛行可能な設計としては引き渡せない。** 次の具体的な設計工程は `design-review.md` に記載した。GitHubへは公開していない。
'''
    (ROOT/'final-report.md').write_text(report,encoding='utf-8')
    files=sorted(p for p in ROOT.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix.lower()!='.fcbak' and p.name!='manifest.sha256')
    manifest=ROOT/'verification/manifest.sha256'
    manifest.write_text('\n'.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.relative_to(ROOT).as_posix() for p in files)+'\n')
    archive=ROOT.parent/'rotor-design-C2.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for p in files+[manifest]:z.write(p,'rotor-design-v2/'+p.relative_to(ROOT).as_posix())
    print(json.dumps({'features':425,'requirements':counts,'width_mm':width,'archive_bytes':archive.stat().st_size},ensure_ascii=False))

if __name__=='__main__':run()

# 中立姿勢の接触台帳

`neutral-overlaps.csv`は145部品の全ての組合せをバウンディングボックスで絞り、交差体積が1 mm³を超える対を保存したもの。**その行を一括で無視して「干渉なし」とは判定しない。** R-25は未検証。

|具体的な対（i=1〜5は同一設計の繰返し）|扱いと不足|
|---|---|
|TiltSocket–Stator|固定接続の一体化領域。現モデルは重ねた支持ウェブ、製作接合詳細なし|
|Stator–Servo0/1/2UpperJoint、Stator–AntirotationB/Joint2|球面ジョイントの取付包絡。実ソケットとピンの穴・フォークの詳細未設計|
|RotorRing–RotatingDriveB/Joint2|駆動ジョイント取付包絡。実回転拘束・ピン詳細未設計|
|RotorRing–Blade i LowerBall/PitchLink|持上げた取付ラグに球面端が入る領域。実ソケットを模しているが加工穴は未設計。回転軸受内への侵入は修正済み|
|ServoBody j–Servo j LowerJoint、ServoRod j–Servo j UpperJoint|端部取付の包絡。シリンダーとロッド本体の交差はボアで除去済み|
|AntirotationA/B/Joint0/1/2、およびRotatingDriveA/B/Joint0/1/2のCSV記載対|腕の中心線が交差するピボット域。実フォーク・回転ピンを未詳細化。自由度と容量を合格とはしない|
|HubRing–Blade i HubArm/DamperMount、Blade i HubArm–OuterRace/DamperMount|固定支持構造の接続域。製法/締結未確定。エラストマー・シムへの侵入は球面逃げで除去済み|
|Blade i InnerRace–Cuff、Cuff–PitchHorn/DamperLug/FoldClevis|一体/固定接合を想定する重なり。溶接/鍛造/締結の選定と強度未確定|
|Blade i PitchHorn–PitchLink/UpperBall、PitchLink–LowerBall/UpperBall|球面ロッド端の包絡。ソケットの外形、ピン/ブーツとミスアライメントの実評価が必要|
|Blade i DamperMount–DamperBody/DA、DamperLug–DamperRod/DB、DamperRod–DB|ダンパー端取付包絡。ボア内ロッドとハウジングの交差は修正済み。実取付構造は未設計|
|Blade i FoldTang–Airfoil/RootSparEnvelope、Airfoil–RootSparEnvelope|複合材翼の等価桁/全体空力外形とインサートの重なり。外形は空間包絡で、中実実部品でも密度積分用でもない。内部積層・接着・締結の設計は必要|

球面支持内外レース間の積層は独立した有限厚さ。回転軸受はボールに合わせた円環溝を作成したが、接触応力と保持容量は未評価。

contact_audit.pyの回帰assertは、支持アームとゴム/シム、リンク下端球と回転軸受、折畳み軸と根元桁、ダンパーのロッドと筒の不正な交差が戻らないことを確認する。全操縦域の非接触を証明する試験ではない。

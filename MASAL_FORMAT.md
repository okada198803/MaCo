# MaSaL交換形式（masal-exchange v1）

MaLI（実弾・ガスブローバックの射撃評価）と MaCo（ドライ・ガスブローバックの撃ち方の診断）の記録を、MaSaL（Marksmanship Synthesis & Analysis Lab）で部隊単位に統合するための共通形式。JSON 1ファイルに、射手の一覧と記録を入れる。波形・軌跡などの生データは含めない。

- MaLI：「2. Session History」の「MaSaL出力」（チェックした記録、なしなら全件）
- MaCo：「振り返り」の「MaSaL出力（このセッション）」「MaSaL出力（全セッション）」
- ファイル名：`masal_<アプリ>_<対象>_<日付>.json`

## 全体

```json
{
  "format": "masal-exchange",
  "version": 1,
  "source": { "app": "MaLI" | "MaCo", "appVersion": "MaLI ver67", "exportedAt": "2026-10-25T12:40:18.237Z" },
  "shooters": [ { "code": "A001", "name": "岡田", "unit": "1中隊/2小隊/3班" } ],
  "records": [ ... ]
}
```

## 射手（shooter）

| 項目 | 内容 |
|---|---|
| code | 記号。MaLI・MaCo・MaSaLで射手を結び付けるキー（部隊で付ける番号など）。空のときは氏名で結び付ける |
| name | 氏名（表示用） |
| unit | 所属。「/」で階層を区切る（例 `1中隊/2小隊/3班`）。記録時点の所属 |

- MaLI：Training Config の名簿（記号・氏名・所属）から選ぶ。所属が空欄なら「所属（既定）」を使う。
- MaCo：設定の「練成者」に記号・氏名・所属を入力する。MaLIの設定ファイル（Training Config の JSON）を名簿として読み込める。

## 記録（record）共通

| 項目 | 内容 |
|---|---|
| id | 一意のID。`MaLI:<セッションID>`、`MaCo:<セッションID>:<練成ID>`、`MaCo:<セッションID>:free:<射手>\|<銃種>`。MaSaLはこれで重複を除く |
| app | `MaLI` / `MaCo` |
| type | `live-session`（MaLIの1セッション）、`drill`（MaCoの練成1回）、`free`（MaCoの練成モード以外の発、セッション・射手・銃種ごと） |
| shooter | 射手（上記） |
| start | 開始日時（ISO 8601、UTC） |
| eventName / eventDate | 訓練名・訓練日（MaLIの Training Config。MaCoは訓練名なし、日付は開始日） |
| condition.weapon | `live`（実弾）/ `gbb`（ガスブローバック）/ `dry`（ドライ） |
| condition.gun | 使用火器・銃種（MaLIは入力値、MaCoは VP9 / Glock17 / Glock17 GBB） |
| condition.scenario | `{ key, label, type }`。type は `string`（ブザー1回で続けて撃つ）、`repeat`（1回ごとにブザー・1発）、`free`（時間制限なし） |
| condition.rangesM | 射距離（m、発ごと）。MaCoは null |
| condition.night | 夜間なら true（MaLIの制圧致命射撃（夜間））。不明は null |
| summary | 記録の要約（下記） |
| shots | 発ごとの結果（下記） |
| actions | 弾倉交換・弾づまり対処の記録 `{ type: reload / malfunction, startS, durS }`（開始ブザーからの秒） |

## summary

### MaLI（live-session）

shots, hits, groupingCm（平均散布界）, groupingsCm（目標ごと）, firstS, splitS, totalS, reloadS, reloadMeasured, malfunctionS, goals（各目標値）, grades（first, split, total, reload, hits, grouping, malfunction, headshot, exposure, adapt：A〜D）

### MaCo drill（string：MaLIの射撃要領など）

shots, plannedShots, scoreMean（予測点の平均。時間外・連射・要確認を除く）, firstS, splitS, totalS, reloadS, reloadMeasured, malfunctionS, goals, grades（first, split, total, reload）, slideS（ドライで目標に加えたスライド操作時間）

### MaCo drill（repeat：1コ的・3コ的など）

reps, shots, inTime, scoreMean, reactMeanS, reactMaxS, cantMaxDeg, parS, restS

### MaCo free

shots, scored, suspect, scoreMean, scoreSd, bands（high/mid/low）, causeCounts（中・低の発の低下要因の件数）, cantMeanDeg

## shots（発ごと）

| 項目 | MaLI | MaCo | 内容 |
|---|---|---|---|
| n | ○ | ○ | 何発目（MaCoの自由練成はセッション内の通し番号） |
| tS | ○ | 練成のみ | 開始ブザーからの秒 |
| splitS | ○ | 練成のみ | 前の発からの秒 |
| target / rangeM | ○ | target | 的番号・射距離 |
| at | － | ○ | 撃発の日時（ISO 8601） |
| score / band | － | ○ | 予測点（±10点程度の幅で読む）・帯（高/中/低） |
| quality | － | ○ | `ok` / `suspect`（要確認：集計から除く） |
| cause | － | ○ | 低下要因 `{ key, label, severity（軽度/中度/重度）, magnitudeDeg, angleDeg }`。方向による候補で断定ではない |
| cantDeg | － | ○ | 銃の傾き（右が正） |
| late / rapid / extra | － | 練成のみ | 時間外／前の発から0.6秒未満（点数は参考）／規定の発数の超過・同じ回の2発目 |
| ref | － | ○ | 参照デバイスの点数・原因（入力した場合のみ） |

## 扱いの注意

- 予測点（MaCo）と命中・散布（MaLI）は尺度が違うため、合算しない。比較は同じ condition（weapon・gun・scenario）どうしで行う。
- 記号が空の記録は氏名で結び付けるため、同姓同名を区別できない。記号の入力を推奨する。
- 個人の記録を含むため、ファイルの受け渡しと保管は部隊の規則に従う。

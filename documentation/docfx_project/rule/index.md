# Rule

&emsp;このページでは本大会のルール・順位付けについて解説します。なお、このページの内容は大会期間中に変更される場合があります。

## Ranking System

&emsp;参加者の順位は以下の2つの指標に基づいて決定されます。

1. 距離点:
   1. 制限時間(10分)内にゴールまで到達できなかった参加者の順位は、制限時間以内に進むことができたスタート地点からの距離に応じた`距離点(0-100%)`により決定されます。
   2. 制限時間内にゴールまで到達した場合、`距離点`は100%です。(簡潔な記載のため詳細な距離の記載は省いています。)
2. 総合タイム:
   1. 制限時間以内にゴールに到達した参加者の順位は、スタートからゴールまでの所要時間にペナルティを加えた、`総合タイム`により決定されます。  

***ランキング例***

| 距離点 | スタートからゴールまでの時間 | ペナルティによるタイム加算 | 総合タイム | 順位 | 
| ------ | ---------------------------- | -------------------------- | ---------- | ---- |
| 100    | 01:10                        | 00:00                      | 01:10      | 1    |
| 100    | 01:30                        | 00:10                      | 01:40      | 2    |
| 100    | 01:20                        | 00:30                      | 01:50      | 3    |
| 60     | N/A                          | 00:00                      | N/A        | 4    |
| 50     | N/A                          | 00:10                      | N/A        | 5    |
| 10     | N/A                          | 00:00                      | N/A        | 6    |

## Goal Position

- 走行完了(ゴール到達)は以下のゴール地点を超えた場合になります。

  ```yaml
    goal.position.x: 21921.96875
    goal.position.y: 51756.328125
  ```
- `goal_pose_setter`を使うことにより、ゴールを設定できます．

## Violations and Penalties

&emsp;走行コースからの逸脱、並走する他車に衝突は反則と判定されます。反則に対する罰則は

1. 走行終了
2. 走行タイムへ10秒の加算
3. 走行タイムへ5秒の加算

の3通りです。以下に、各反則とそれに対応する罰則を記します。

**走行終了**

- コース境界から5m以上離れた場所を10秒間以上逸脱した場合
- コース境界から100秒以上離れた場合

**重度な反則(ペナルティ: 10秒/回):**

- 他車ととの衝突状態が2秒間以上継続した場合
- コース境界から5m以上離れた場合
- コース境界から2秒以上離れた場合

**軽微な反則(ペナルティ: 5秒/回):**

- コースから逸脱する(2秒未満 **かつ** コース境界から5m未満)
- 他車と衝突する(衝突状態を2秒以内に解消する)

**注意**

- 自車の後部に他車が衝突するケース(例: 他車による追突)は自車の責任ではないとしペナルティは適用されません。
- 逆走はしないでください。
- コーナーでのカットインはしないでください。

## Submission

&emsp;参加者の皆様には、開発したソフトウェアを提出用のページから評価システムにアップロードしていただきます。1回のアップロードにつき3回のシミュレーションが行われ、各シミュレーションに対して距離点、総合タイムの計算が行われます。3回のシミュレーション結果のうち、最も高いスコアを記録した結果がランキングに反映されます。

### How to check results

&emsp; 結果のスコアは`result.json`に送られます。
#### 結果のログ形式
&emsp; 結果は`~/awsim-logs/result.json` に以下のフォーマットで出力されます。

```json
{
  "rawLapTime": 72.77926,
  "distanceScore": 86.7,
  "lapTime": 302.779266,
  "isLapCompleted": false,
  "isTimeout": false,
  "trackLimitsViolation": [
    19, # Duration of time spent off track is less than 2 seconds. (Minor)
    19, # Distance from track limit is less than 5 meters. (Minor)
    2,  # Duration of time spent off track is more than 2 seconds. (Major)
    2,  # Distance from track limit is more than 5 meters. (Major)
    0   # not used
  ],
  "collisionViolation": [
    0, # Duration of collision is less than or equal to 2 seconds. (Minor)
    0, # Duration of collision is more than 2 seconds. (Major)
    0, # not used
    0  # not used
  ]
}
```

&emsp; また、その他のログとして`~/awsim-logs/verbose_result.json`も以下の形式で出力されます。

```json
{
  "rawLapTime": 72.77926,
  "distanceScore": 86.7,
  "lapTime": 302.779266,
  "isLapCompleted": false,
  "isTimeout": false,
  "boundsViolations": [
    {
      "distance": 0.3017645,
      "distanceFromBound": 2.26600266,
      "duration": 0.0160293132
    },
    {
      "distance": 2.776487,
      "distanceFromBound": 1.01412094,
      "duration": 0.0801174641
    },
    {
      "distance": 2.91162729,
      "distanceFromBound": 1.1498549,
      "duration": 0.08674298
    },
    ....
  ]
  "collisionViolations": []
}
```

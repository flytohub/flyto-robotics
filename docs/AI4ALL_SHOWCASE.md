# Flyto2 AI4ALL 多設備 Physical AI Demo

## 一句話

Flyto2 把臨時任務轉成經原子能力、設備範圍與安全政策驗證的執行計畫；現場設備或感知來源改變時，系統能切換到已宣告的備援資源，而不是讓 LLM 直接控制馬達。

## 這次只證明一個重點

同一個院內配送任務，依序遇到：

1. 路網先分成黃／橘兩路，匯流後再展開藍／綠／紫／紅四路，形成八條完整候選路徑。
2. 確定性依賴策略先依距離、設備健康、證據要求與替代條件排序；Flyto AI 只看通過硬條件的候選與原子 shortlist。
3. 第一輪真實模型選出 `黃→紫`，每個模型回合都有 provider、model、request／plan／schema 雜湊與延遲證據。
4. 執行前攝影機 B 健康檢查失敗，四條黃線路徑全部被排除；第二輪模型只能從四條橘線路徑重新選擇。
5. 選定路徑被約束式 Schema 展開為完整語意位置序列、`ask_human`、`resume`、`safe_stop`，不能跳站、混路或直接輸出馬達欄位。
6. Gazebo 依同一份 attested plan 執行；動態障礙物放到機器人當下路徑前方，LiDAR 觸發安全停止，障礙移除後才恢復。
7. 護理站核准完成後才繼續；重播同一簽章 nonce 會被拒絕。設備 handoff、喇叭端點與影片影格都留下同一回合的可重播證據。

這不是「多播放幾段影片」。三台攝影機都是 Gazebo 的獨立感測器，切換決策依機器人的 ground-truth 區域與健康狀態產生；障礙物則透過 Gazebo service 實際改變模型位置。語意地圖使用機器人里程計座標，彩色路網使用 Gazebo world 座標；測試固定驗證 `odom_x + (-2.15 m world origin) = route world_x`，避免計畫名稱選橘線、車體卻跑在另一條視覺路徑上的座標框架錯置。

## AI 與控制責任

| 層 | 負責 | 不負責 |
|---|---|---|
| Flyto AI | 理解目標、從 shortlist 組合原子、提出失敗策略 | PWM、輪速、ROS topic |
| Capability Router | 依 Goal Frame、能力、資源與權限縮小候選 | 自由生成不存在的工具 |
| Validator | 拒絕未知原子、危險參數、缺少安全停止與清單外能力 | 現場即時馬達控制 |
| Executor | 確定性執行、暫停、恢復、重試與事件記錄 | 猜測未宣告設備 |
| ROS 2 / 控制器 | 里程計、LiDAR、速度限制、零速停止 | 語意任務規劃 |

## 設備依賴不是固定五級

Flyto2 不把設備永久標成弱、中或重依賴。依賴屬於「某個工作流程在某個
階段如何使用這台設備」，不是設備種類本身。一般攝影機拿來觀看畫面時可以
安全降級；同一台攝影機若是電梯門狀態的唯一安全來源，就必須安全停止。

底層合約分開保存以下事實：

| 軸 | 表達的問題 |
|---|---|
| `safety_impact` | 資源失效時，移動是否可繼續、暫停、停止或終止 |
| `task_impact` | 任務可忽略、降級、阻塞，或整個結果失效 |
| `evidence_requirement` | 影像／量測是輔助紀錄，還是交付成立的必要證據 |
| `substitution_mode` | 任意健康設備、等價設備、已驗證設備，或完全不可替代 |
| `minimum_confidence` | 感知結果最低可信度；不等於依賴強度 |
| `max_age_seconds` | 感知資料可接受的新鮮度 |
| `recovery_timeout_seconds`、`retry_limit` | 可以等待與重試多久 |
| `active_phases` | 依賴只在哪些任務階段有效 |

執行器依上述事實推導 `continue_degraded`、`switch_substitute`、
`pause_and_escalate`、`safe_stop_and_escalate` 或
`safe_stop_and_abort`。UI 可以再把結果簡化顯示為「輔助、必要、安全關鍵、
任務關鍵」，但這些是衍生提示，不是限制底層能力的固定級數。

設定採三層覆寫：

1. 設備註冊提供最小安全預設；一般觀看用攝影機預設為可替代的觀察來源。
2. 每張工作流程卡片可依用途提升安全、任務、證據或替代要求。
3. 執行時另外讀取健康、可信度、資料年齡與備援驗證狀態，不讓 LLM 自行猜測。

基礎 UI 只需顯示「自動」與系統推導結果；開啟進階設定後，才呈現各軸、
門檻、作用階段與復原時間。使用者不調整時仍有確定性預設，專業場域也不會
被單一等級綁死。

## 為什麼比預設好路線的機器人更強

- 任務順序是可重組的原子計畫，不是固定程式分支。
- 設備名稱、區域、優先序與 fallback 都在版本化資源檔，不寫死在 LLM prompt。
- 攝影機失聯不會讓模型自由猜另一台設備；路由器只會選擇該 AI Space 已宣告且健康的端點。
- 換機器人或攝影機時保留任務、驗證、UI 與證據格式，只替換 Adapter／SDK。
- 每一步都能回答「AI 選了什麼、Validator 為何允許、Executor 實際做了什麼、感測器看到什麼」。

## 執行

先啟動一個 Flyto AI 結構化規劃端點；例如在 `flyto-ai` repository：

```bash
python3 -m flyto_ai.robotics_planner_server \
  --model flyto-qwen3:8b
```

再執行：

```bash
FLYTO_ROBOTICS_PLANNER_URL=http://127.0.0.1:8787/v1/robotics/plan \
  make ai4all-showcase
```

同一套原子能力也提供「進階安全交付」模式；它保留原本的分叉選路、
攝影機切換、障礙停止與簽章重播防護，再加入可重播的交付閘門：

```bash
FLYTO_ROBOTICS_PLANNER_URL=http://127.0.0.1:8787/v1/robotics/plan \
  make ai4all-medication-showcase
```

此示範使用明確標示為 synthetic 的 12 號病人、藥袋 `A12` 與錯誤藥袋
`B13`，不含真實個資。流程先驗證批價，再刻意掃描 `B13`；錯藥事件會被
拒絕且箱體保持上鎖。掃描 `A12` 後仍不能直接跳到交付，必須從
`verify_item` checkpoint 恢復。抵達後先以錯誤的 `patient-13` 驗證，
箱體仍保持上鎖；只有 `patient-12` 通過，控制器才可執行解鎖。每個原子
都記錄前後狀態、操作者、鎖定狀態與順序。

進階模式不是寫死的藥品機器人。底層使用通用
`check_precondition`、`scan_item`、`resume_checkpoint`、
`scan_recipient`、`unlock_container`、`complete` 原子與版本化 policy；
替換 policy、Adapter 或控制器，就能套用在耗材、文件、檢體或其他需
受控交付的設備流程。LLM 負責組合已允許的能力，確定性狀態機負責
fail-closed 安全邊界。

腳本會先取得兩輪真實模型結果並完成獨立驗證，之後才啟動
Gazebo。規劃端點不可用、模型兩次皆不合格、attestation 不符、
故障未排除原路線，或最終 plan 與執行 plan 不同時都會直接失敗；
沒有自動退回預寫 fixture 的路徑。

執行會建立未追蹤的 `results/ai4all-showcase/<run-id>/`，包含：

- `planning-session.json`：八選一、設備失效、四選一的兩輪完整請求、回應與 attestation。
- `validated-plan.json`：唯一允許交給 Gazebo 的最終原子計畫。
- `gazebo-active-camera.mp4`：依租約切換的 A、B、備援攝影機真實影格。
- `gazebo-overhead.mp4`：同一次模擬的全局對照。
- `mission-result.json`：原子執行結果與 22 筆連續任務事件。
- `facility/showcase-evidence.json`：能力 shortlist、LLM 計畫、設備健康、租約與 handoff。
- `showcase-report.json`：16 項端到端判定。
- `lab-report.json`：26 項分叉 Gazebo 安全與物理判定。
- `videos.sha256`：影片內容雜湊。

進階安全交付回合的 `showcase-report.json` 會增加到 21 項判定；新增的
五項分別檢查前置條件、錯藥阻擋與 checkpoint、錯誤收件者阻擋、所有
閘門通過後才解鎖，以及完成交付狀態機後才發布任務核准。簡單模式仍維持
16 項，不必承擔未使用的設定與畫面複雜度。

使用核准的 Flyto2 Logo 產生中文解說版：

```bash
FLYTO2_LOGO_FILE=/absolute/path/to/flyto2-logo.png \
  scripts/render-ai4all-showcase-video.sh \
  results/ai4all-showcase/<run-id>
```

輸出的 `flyto2-ai4all-showcase.mp4` 同時使用真實全局 Gazebo
攝影機與目前租用的分區攝影機。淺色證據控制台會依時間顯示兩輪 Qwen
選路、8→4 候選淘汰、實際 plan hash、LiDAR 停止／恢復、攝影機
A→B→overhead handoff、核准與 nonce 重播拒絕。結尾保留三秒真實完成影格，
讓 16/16、26/26、5.526 公尺累積路徑與一次安全停止可以閱讀；所有數值均
來自同一回合報告，不重畫模擬畫面。

若輸入回合包含 `guarded_handoff.enabled=true`，同一支渲染腳本會自動選擇
進階安全交付版，輸出 `flyto2-ai4all-medication-showcase.mp4`。錯藥、
checkpoint、錯誤收件者、正確收件者、解鎖與完成字幕的開始／結束時間，
都直接讀取該回合 `driver-manifest.json` 的 `at_seconds`，不是依影片長度
猜測或手工對時。影片同時標示 21 項端到端判定、28 項 Gazebo 驗收、
實測路徑、LiDAR 停止與 9 筆安全交付事件。

2026-07-31 的進階接受回合是
`results/ai4all-showcase/medication-handoff-live-v8/`。該回合由
`flyto-qwen3-8b` 完成兩輪真實規劃（53.087 秒與 33.919 秒），第二輪選定
`orange-purple`。Gazebo 任務於 28.0 秒完成，量得 5.212 公尺位移與
5.526 公尺累積路徑；`B13` 與 `patient-13` 均在箱體上鎖時被拒絕，
27.0 秒才解鎖，27.7 秒完成交付狀態機後才發布核准。端到端與 Gazebo
報告分別為 21/21、28/28。

2026-07-31 接受的閉環證據回合是
`results/ai4all-showcase/simple-delivery-qr-live-v7/`。該回合使用
`flyto-qwen3-8b` 進行兩次真實結構化規劃，Gazebo 任務在 22.4 秒完成，
產生 22 筆連續任務事件；獨立 world-pose 量得 5.212 公尺位移、5.526
公尺累積路徑。QR 簽章核准成功，同一 QR nonce 與轉換後人員決策 nonce
的重播皆被拒絕，原始 QR token 未寫入證據。最終中文證據影片為
1920×1080、30 fps、25.267 秒，共 758 格。此前 v1、v2、v3 分別暴露
舊驗收條件、座標框架錯置與終點安全距離問題；後續回合補齊 QR 核准、
重播防護與影片資訊，只有 v7 是目前接受的完整證據。

## 連續 Gazebo GUI 驗證影片

評審用驗證片以真實 Gazebo Sim 視窗為主畫面，保留場景、機器人、動態
障礙、時間與 Gazebo GUI 的同一段連續時間線。它不使用生成式影像，也
不以投影片、重畫路線或交叉轉場代替模擬。字幕只從該回合的
`planning-session.json`、`validated-plan.json`、`mission-result.json` 與
`driver-manifest.json` 產生，因此顯示的模型、候選數、路線、plan hash、
障礙停止、交付拒絕與解鎖時間都有原始 JSON 可核對。

先用已通過完整驗收的回合作為不可變規劃輸入，再啟動一次有視窗的相同
Gazebo 任務：

```bash
scripts/run-ai4all-gui-evidence.sh \
  results/ai4all-showcase/medication-handoff-live-v8
```

腳本使用 Xvfb 錄製完整 Gazebo GUI，不抓取預先輸出的圖片序列。它會將
原回合的 attested plan 與 planning session 複製到新的未追蹤回合，重新
執行 ROS 2／Gazebo，產生新的 mission、driver、lab 與 showcase 報告；
任何必要檔案缺失、GUI 沒有出現或驗收失敗都會結束為非零狀態。
`gui-capture-metadata.json` 以任務完成時間與實測 elapsed time 計算 GUI
錄影中的任務起點，使事件字幕與同一回合的模擬時間對齊。

再以核准 Logo 產生單一連續的中文驗證片：

```bash
FLYTO2_LOGO_FILE=/absolute/path/to/flyto2-logo.png \
  scripts/render-ai4all-verification-video.sh \
  results/ai4all-showcase/<gui-capture-run-id>
```

輸出 `flyto2-gazebo-verification.mp4`。右上角持續顯示真實模型名稱、
8→4 候選排除、原路線與重規劃路線、plan hash；下方事件列只在因果狀態
改變時更新。觀眾會在同一畫面中看到障礙進入、車體零速停止、障礙移除、
沿橘—紫路線恢復、錯藥與錯誤收件者保持上鎖，以及所有條件成立後才解鎖。
右上角保留原始未裁切 Gazebo GUI，旁邊的實際事件 LOG 則由
`planning-session.json`、`mission-result.json` 與 `driver-manifest.json`
直接產生，可在同一幀核對 AI 規劃、感測距離、checkpoint 與箱體鎖定狀態。
`verification-video-probe.json` 與 `verification-video.sha256` 可供第三方核對
編碼規格與影片／計畫／任務／driver 證據鏈。

## 誠實邊界

模型規劃證據來自設定的真實規劃端點；`deterministic_fixture` 不得標成
`live_llm`。Rover、三台攝影機、障礙物與物理位移來自 Gazebo
Harmonic 模擬，不是實體醫院或實體機器人。攝影機與障礙故障是受控測試
注入；喇叭只驗證端點選擇與租約，尚未宣稱實體播音。實機部署使用相同
任務與資源契約，但仍需完成硬體 E-stop、真實網路失聯與場域驗收。

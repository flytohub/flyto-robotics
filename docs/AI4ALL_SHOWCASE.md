# Flyto2 AI4ALL 多設備 Physical AI Demo

## 一句話

Flyto2 把臨時任務轉成經原子能力、設備範圍與安全政策驗證的執行計畫；現場設備或感知來源改變時，系統能切換到已宣告的備援資源，而不是讓 LLM 直接控制馬達。

## 這次只證明一個重點

同一個院內配送任務，依序遇到：

1. AI 從完整能力登錄表縮小候選範圍，並組合 `navigate`、`wait_until_clear`、`ask_human`、`resume`、`safe_stop`。
2. 機器人在藍色區域使用走廊攝影機 A。
3. 進入黃色區域後，釋放 A 並綁定攝影機 B。
4. Gazebo 動態障礙物進入 LiDAR 路徑，機器人安全停止；障礙移除後才恢復。
5. 進入紫色區域時注入攝影機 B 失聯，資源路由器切換到已宣告的全樓層備援攝影機。
6. 護理站核准完成後才繼續；重播同一簽章 nonce 會被拒絕。
7. 抵達後綁定護理站喇叭端點，並保存完整事件、影格、位移與雜湊證據。

這不是「多播放幾段影片」。三台攝影機都是 Gazebo 的獨立感測器，切換決策依機器人的 ground-truth 區域與健康狀態產生；障礙物則透過 Gazebo service 實際改變模型位置。

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

```bash
make ai4all-showcase
```

執行會建立未追蹤的 `results/ai4all-showcase/<run-id>/`，包含：

- `gazebo-active-camera.mp4`：依租約切換的 A、B、備援攝影機真實影格。
- `gazebo-overhead.mp4`：同一次模擬的全局對照。
- `mission-result.json`：原子執行結果與 30 筆任務事件。
- `facility/showcase-evidence.json`：能力 shortlist、LLM 計畫、設備健康、租約與 handoff。
- `showcase-report.json`：12 項端到端判定。
- `lab-report.json`：28 項原有 Gazebo 安全與物理判定。
- `videos.sha256`：影片內容雜湊。

使用核准的 Flyto2 Logo 產生中文解說版：

```bash
FLYTO2_LOGO_FILE=/absolute/path/to/flyto2-logo.png \
  scripts/render-ai4all-showcase-video.sh \
  results/ai4all-showcase/<run-id>
```

輸出的 `flyto2-ai4all-showcase.mp4` 保留連續的真實 Gazebo
感測器影格，只疊加已由報告驗證的 AI shortlist、Validator 邊界、設備
handoff、故障備援與量測結果；它不是重畫的模擬動畫。

## 誠實邊界

目前 rover、三台攝影機、障礙物與物理位移為真實 Gazebo Harmonic 模擬。攝影機 B 失聯是測試驅動器依實際區域注入的故障；喇叭只驗證端點選擇與租約，尚未宣稱實體播音。實機部署使用相同任務與資源契約，但仍需完成硬體 E-stop、真實網路失聯與場域驗收。

# Flyto Robotics 產品介紹

## 一句話

Flyto Robotics 是一套「AI 原生的機器人能力作業系統」：使用者說出目標，
AI 從機器人目前擁有的原子能力中自行選擇、排序與組合，安全執行後再根據
現場觀察決定完成、恢復、詢問人類或重新規劃。

它不是循線機器人，也不是把 LLM 接到馬達上。藍線、黃線、紫線只是第一個
容易看懂的驗證案例。

## 使用者得到什麼

使用者可以說：

> 先沿藍線到倉庫，看到破損箱子就拍照，接著沿黃線到工作站；如果工作站
> 有人就先詢問，沒人則放下物品，最後沿紫線回充電座。

AI 不需要收到一份手寫的單一路線程式。它會取得當前機器人的能力目錄，
例如：

- `navigate`：到一個位置；
- `follow_line`：沿指定視覺路徑移動；
- `observe`：取得相機、雷達、力覺或其他感測結果；
- `detect`：辨識物體、標誌、缺陷或人員；
- `approach`、`grasp`、`place`：接近、抓取、放置；
- `speak`、`ask_human`：說話或要求人類確認；
- `run_tool`：呼叫受控的 C、C++、Python、ROS action 或設備工具；
- `safe_stop`、`dock`、`charge`：停止、回座、充電；
- `retry`、`request_replan`：恢復或請 AI 重新規劃。

每一項能力都有版本、輸入欄位、數值範圍、需要的感測器、安全等級與可能
產生的事件。AI 可以組合能力，但不能憑空發明未安裝的能力。

## 核心閉環

```text
自然語言目標
   ↓
機器人能力目錄 + 現場觀察
   ↓
AI 規劃：選能力、排順序、填受限參數、設定失敗策略
   ↓
Plan Schema + Capability Registry 安全驗證
   ↓
不可變 WorkflowPlan
   ↓
確定性即時控制器
   ↓
ROS 2 → Gazebo 或真實機器人
   ↓
事件、結果、影像、感測與失敗原因
   └──────────────→ AI 視需要重新規劃
```

這裡有兩個不同時間尺度：

1. AI 的慢迴圈負責「要做什麼」：語意理解、任務拆解、能力選擇、順序、
   路線、有限範圍內的速度與策略，以及失敗後的新計畫。
2. 安全控制器的快迴圈負責「此刻怎麼動」：相機偏差修正、輪速、轉向、
   速度上限、雷達避障、感測器逾時與緊急停止。

LLM 不直接輸出 PWM，也不在 10 Hz 控制迴圈裡臨場猜下一個馬達值。這讓
產品同時保有 AI 的泛化能力與機器人需要的可預測性。

## 與 Flyto 產品線的關係

- Flyto Cloud 管理使用者、組織、設備、排程、模型／Agent 入口、任務派送
  與執行證據。
- Flyto Robotics 管理具身能力目錄、AI plan 信任邊界、機器人端工作流、
  ROS 2／設備 adapter、模擬與實體執行。
- 既有 Flyto 執行環境能處理 Python、C 或其他程式；在 Robotics 裡，
  這些程式不必變成整台機器人的單體應用，而能被包成具有明確契約的能力
  atom，供 AI 重複組合。

因此 Flyto Robotics 可以獨立發展，不需要把所有機器人邏輯塞進
Flyto Cloud；兩者透過版本化 JSON plan、job、result 與 evidence 交接。

## 目前已完成的底

目前 repository 已具備：

- 通用 `CapabilityRegistry` 與機器可讀的能力目錄；
- `flyto.robotics.plan.v1` AI 計畫契約；
- 對 LLM 輸出做大小、欄位、能力、參數、條件與 robot identity 驗證；
- 跨步驟安全政策：移動計畫必須以 `safe_stop` 結束、彩線交接必須一致、
  人員核准與恢復必須成對；
- provider-neutral planner request；
- 可接 Flyto Cloud 或本地服務的 HTTPS JSON planner adapter；
- `navigate`、`navigate_to_location`、`save_current_location`、
  `follow_line`、`dwell`、`wait_until_clear`、`ask_human`、`resume`、
  `safe_stop` 九個可執行 atom；
- 相機彩色路徑感知、lidar 障礙停止、odometry 與感測器 freshness；
- 外部具名 actor 核准、fail-closed 恢復檢查，以及不含敏感文字的
  `prompt_key`；
- `flyto.robotics.human-decision.v1` 短效核准契約，具有 HMAC-SHA256
  簽章、job／robot 綁定、到期時間與 nonce 防重播；
- 只在包含 `ask_human` 的 workflow 開啟 `/flyto/human_decision` ROS 2
  訂閱，未驗證訊息不會進入 MissionController；
- `abort` 與 `request_replan` 失敗策略，以及帶有 sequence、step、
  capability、actor 的結構化事件紀錄；
- ROS 2 Jazzy、Gazebo Harmonic、可替換實體底盤的 adapter 邊界；
- 醫院配送世界與藍／黃／紫路線世界；
- 確定性乾跑、契約驗證、單元測試與機器可讀 result；
- 對抗式 Gazebo lab：動態障礙注入、真實 LiDAR 停車、簽署核准、nonce
  重播拒絕、四張 overhead 圖、Gazebo 獨立 ground truth；
- JSON、Markdown、JUnit、輸入雜湊、50 次 soak 與三次 cold-start matrix。

目前的實測成熟度分成兩級：

- AI 編排 `route.blue → route.yellow → route.purple → safe_stop`，透過
  `navigate` waypoint atom 在 ROS 2 Jazzy／Gazebo Harmonic 真實物理中
  已完整成功，並產出版本化 result。
- `follow_line` 已完成相機 topic、RGB 感知、ROI 車體遮罩、控制命令、
  障礙停止、timeout 與 `request_replan`；確定性 observation 乾跑已成功。
  正下視相機的完整物理 segment handoff 尚未列為通過，曲線版需要加入
  線段朝向估計，而不是只使用橫向質心。

`examples/plans/careflow-human-gate.json` 是智慧醫院閉環的確定性驗收計畫：

```text
follow_line(blue)
→ wait_until_clear
→ follow_line(yellow)
→ ask_human(delivery.nurse_station)
→ resume(delivery.nurse_station)
→ follow_line(purple)
→ safe_stop
```

乾跑會刻意注入障礙並由 `demo.operator` 提交核准。結果必須同時包含
`clearance_blocked`、`human_approval_requested`、`human_approved` 與
`resume_authorized` 證據，否則不算通過。這是可重播的核心控制測試；
目前尚未宣稱 ROS 2 現場操作介面已完成身分驗證或醫院部署安全認證。

現在已加入可在 Gazebo／ROS 2 使用的簽署核准 transport。這代表任意 ROS
publisher 無法只靠填入 `actor_id` 讓任務恢復；訊息還必須通過共享密鑰
簽章、任務與機器人綁定、短效期限及 nonce 防重播。不過 HMAC 證明的是
「可信 signer 發出」，不是完整的醫院使用者登入與 RBAC。正式部署時，
共享密鑰只能由 Flyto gateway 持有，由 gateway 在完成登入、權限與核准
政策後代為簽署，不能發給一般操作人員。

AI 規劃服務可以用下列方式接入：

```bash
export FLYTO_ROBOTICS_PLANNER_URL=https://your-flyto-planner/robot-plan
export FLYTO_ROBOTICS_PLANNER_TOKEN=...

python3 -m flyto_robotics.cli plan-ai \
  --goal "先走藍線，再走黃線，最後走紫線並安全停止" \
  --robot-id flyto-rover-sim-001 \
  --output results/plan.json
```

回傳結果通過驗證後才能交給 ROS 2 executor。API token 只從環境讀取，不會
寫入 plan 或 result。

## 智慧醫院 CareFlow MVP

競賽產品不以「會沿彩色線走」為核心，而以「醫院人員可以用自然語言建立
安全、可核准、可稽核的機器人流程」為核心。第一個情境鎖定耗材與文件
運送，不處理藥品、檢體或臨床決策。

MVP 必須證明六件事：

1. 自然語言能轉成嚴格 Plan JSON；
2. Capability Registry 能證明目前機器人是否具備所需能力；
3. Validator 會拒絕未知能力、越界參數、矛盾路線、孤立 `resume` 與缺少
   終端 `safe_stop` 的移動計畫；
4. Executor 能組合移動、等待淨空、人員核准、恢復與安全停止；
5. 感測結果或低信心狀態能使機器人保持零速度、失敗或請求重新規劃；
6. 每一步都留下可依序重播的 actor、step、capability 與結果證據。

物理 Gazebo 驗收使用 `careflow-waypoints-human-gate.json`，避開尚未穩定的
相機曲線交接，把測試焦點鎖定在 ROS 執行、感測淨空、簽署核准、恢復與
安全停止。彩線相機版則繼續作為感知控制里程碑，兩者證據不得混稱。

這條物理驗收已在 Linux ARM64、ROS 2 Jazzy 與 Gazebo Harmonic 跑通。
最新嚴格 lab 在 18.9 模擬秒完成，真實 LiDAR 障礙停車 1 次，30 筆事件
序號連續，最終位置 x=4.2618；Gazebo 自己回報的 world displacement 為
4.246871 m，避免只看輪子 odometry 的假成功。28/28 個 assertion 通過。

另外三次完全獨立的 Docker／ROS／Gazebo cold start 全數通過，每次都是
28/28；位移範圍 4.241826–4.247524 m，完成時間 18.900–19.001 秒。
確定性閉環也連跑 50 次，50/50 成功且只有一個 normalized fingerprint。
完整數據、失敗修正歷史與圖片索引在
`docs/testing/TEST_RESULTS_2026-07-29.md`。

Flyto Cloud 的資料夾 AI Space 可作為這套能力的管理入口：資料夾限定可見
的 workflow／MCP、裝置、記憶、選擇政策與 Forge 草稿。AI 先從 Space 的
已驗證 workflow 選擇；缺少能力時產生可審核的新 atom／workflow 提案，
經驗證與 Gazebo 模擬後才發布。這比每次把所有 atom 全塞給 LLM 更穩定，
也保持 atom 本身語系中立。

彩色路線是觀眾看得懂的視覺語言，不是固定業務邏輯。換成 QR marker、
AprilTag、地圖 waypoint、語意導航或廠商 AMR API 時，AI plan 與安全閘門
仍然可以保留，只替換 capability adapter。

## 為什麼它有產品差異

一般做法常落在兩端：一端是每個場景都重寫程式的傳統自動化；另一端是把
LLM 直接放進控制迴圈、看似聰明但難以驗證。Flyto Robotics 的差異在中間
這一層：

- 原子能力可以跨情境重用；
- AI 真正決定組合，而不是只替固定流程填字；
- Registry 是執行時 allowlist，不只是一段 prompt；
- 計畫、參數、事件、失敗與模型來源都有版本且可稽核；
- 高階 AI 與低階 safety controller 明確分離；
- 同一計畫介面可由 Python、C/C++、ROS、廠商 SDK 或遠端工具實作；
- 先在 Gazebo 驗證，再替換為實體 adapter，減少硬體試錯成本。

## 可發展的產品場景

- 倉儲：巡架、盤點、找異常箱、搬運、回充；
- 工廠：設備巡檢、熱點辨識、讀表、取放、人工確認；
- 醫療與照護：物資配送、樣本搬運、遠距巡房輔助；
- 實驗室：依條件取樣、儀器操作、拍照、等待結果後續跑；
- 商場與場館：導引、巡邏、事件回報、多區域任務；
- 農業與戶外：沿區域巡檢、辨識、標記、噴灑或採樣；
- 教育與競賽：讓參賽者新增 atom，而不是重做整套機器人。

## 下一階段

下一批最有價值的 atom 是 `observe`、`detect_object`、`approach`、
`grasp`、`place`、`speak`、`dock` 與受沙箱限制的
`run_tool`。接著要加入 capability 版本協商、前置／後置條件、資源鎖、
平行分支、成本估計、記憶與多次 replan，讓 AI 不只排直線步驟，而能產生
條件式、可恢復、可部分平行的具身工作流。

## 30 秒介紹

Flyto Robotics 讓你不用替每個機器人任務重新寫一個單體程式。你只要把
感知、移動、抓取、說話、C/Python 工具與安全動作註冊成原子能力，AI 就能
依自然語言目標和現場狀況，自行組成受驗證的任務流程。AI 負責理解與
重規劃，確定性控制器負責即時運動與安全；同一套 plan 可以先跑 Gazebo，
再換到真實 ROS 2 機器人，並把每一步證據回傳 Flyto Cloud。

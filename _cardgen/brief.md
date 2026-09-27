# 任务：为 HC32F460 FOC 学习板的某个 mode 生成"模式速览卡"数据（JSON）

## 你的产出
每个 mode 写**一个 JSON 文件**到 `D:\WS_L_re\_cardgen\cards\mode<NN>.json`（NN 带前导零，如 `mode26.json`）。
**不要修改任何源码文件**，只写这一个 JSON。

## 背景
`ws/` 目录下是 BLDC/FOC 学习板固件，每个 mode 一个 `foc_<NN>_<短名>.c/.h`。
mode 40 的卡片已经定稿，作为**格式与详略程度的黄金样板**，它的渲染结果在：
`D:\WS_L_re\_cardgen\out\mode40.txt` —— **先读它**，照着它的信息密度和语气写。

ws 目录绝对路径：
`D:\WS_L_re\ws_L_v.inmop_curloop_re\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws`
main.c 绝对路径：
`D:\WS_L_re\ws_L_v.inmop_curloop_re\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\template\source\main.c`
已机械清点好的清单（VOFA 填充函数体、每个头文件的 extern 符号+行号、宏定义+行号、main.c 派发行）：
`D:\WS_L_re\_cardgen\inventory.txt`
实测结论可能记录在：`D:\WS_L_re\md_record\` 目录（用 grep 搜 mode 号或文件名关键词）。

## 铁律（违反会让编译/上机出错）
1. **只写你在代码里真正看到的符号名**。每个 `name` 必须逐字来自 `.h` 的 `extern` 声明或 `.c` 的定义。
   绝对不要凭印象补名字、不要"应该是叫这个"。查不到就**不写这一条**。
2. 每个写进 JSON 的符号名，都要在 `evidence` 里给出出处，格式 `"符号名": "文件名:行号"`。
3. VOFA 通道数量、顺序、单位**必须**和该 mode 的填充函数逐行一致（`return N;` 就是通道数）。
   填充函数名本身也要进 `evidence`。
4. 只用 ASCII 标记：`[!]` `(*)`。**禁止**出现 `—` `→` `≥` `°` `×` `·` `★` `①` 这类宽度不确定字符
   （在 Keil 里宽度算不准会让整列错位）。要写就用 `--` `->` `>=` 代替。
5. 标点用中文全角（，。；：（））没问题，它们是确定宽字符。数字/单位/符号名保持 ASCII。
6. 描述要**短**。显示宽度 = 中文/全角字符算 2 列、ASCII 算 1 列。目标上限：
   - `watch[].desc` ≤ 26 列；`obs[].desc` ≤ 40 列；`vofa[].desc` ≤ 34 列；`vofa[].what` ≤ 24 列。
   超一点没关系（生成器会自动折行并对齐），但不要写成长句。
7. **不要自己排版**，不要输出 ASCII 表格，只给结构化 JSON，排版由生成器负责。

## JSON 格式（严格按此结构，键名不许改）
```json
{
  "mode": 26,
  "title": "开环强制运行（一句话说清这个 mode 干什么）",
  "entry": "comm_mode = 26",
  "pre": "进入前必须先做什么；没有前置就写“无”",
  "end": "怎么停；自动停就写清自动停条件",
  "watch": [
    { "mk": "", "name": "g_olf_target_v", "val": "3.0", "unit": "V", "desc": "目标电压（写它即给给定）" },
    { "mk": "(*)", "note": "紧跟其后的说明/实测结论，可折行" },
    { "mk": "[!]", "note": "易错点" }
  ],
  "obs": [
    { "mk": "", "name": "g_olf_ialpha", "desc": "一句话说清它是什么、看它能判什么" },
    { "mk": "[!]", "note": "可选：与上一批变量相关的易错点" }
  ],
  "vofa": {
    "n": 16,
    "fn": "Foc_Olf_VofaFill",
    "shared": false,
    "rows": [
      { "mk": "", "ch": "ch0", "what": "U 相电流", "unit": "A", "desc": "" }
    ]
  },
  "notes": [
    { "mk": "(*)", "text": "主判据：怎么判断这个 mode 跑对了" },
    { "mk": "[!]", "text": "坑 / 常见误判" }
  ],
  "evidence": {
    "g_olf_target_v": "foc_26_olf.h:88",
    "Foc_Olf_VofaFill": "foc_26_olf.c:380"
  }
}
```

### 字段说明
- `watch`：**可从 Keil Watch 直接改**的变量/结构体成员。`val` 写代码里的默认值（宏或初始化值），
  查不到就留空字符串 `""`。`unit` 没有单位写 `"-"`。
  结构体成员写全名，例如 `g_olf_pid_cfg.kp`。
- `obs`：**只读**的关键观察变量，每条一行；`name` 可以写两个相关变量用 ` / ` 连接
  （如 `g_olf_vd / g_olf_vq`），此时 `evidence` 每个都要有。
- `vofa`：
  - 该 mode **有自己**的填充函数：`shared = false`，`fn` 写真名，`n` 写 `return` 的值，
    `rows` 按 ch0..ch(n-1) **一条不漏、顺序一致**。
  - 该 mode **没有**自己的填充函数（走通用布局）：`shared = true`，`fn = "Foc_Common_VofaFill"`，
    `n = 20`，`rows` 抄 main.c 里 `Foc_Common_VofaFill` 的 20 个通道（读 main.c 第 151 行那个函数）。
  - `rows[].mk` 只给主判据通道标 `(*)`，其余留空。
- `notes`：2~5 条"判据与坑"，这是卡片最有价值的部分。内容要有依据（代码结构、宏、
  `md_record/` 里的实测记录）。没依据就别编。
- `evidence`：所有符号名 -> `文件:行号`。文件名不带路径，行号是符号**声明/定义**所在行。

## 工作步骤（请照做，不要跳步）
1. 读 `D:\WS_L_re\_cardgen\out\mode40.txt`（黄金样板）。
2. 读 `D:\WS_L_re\_cardgen\inventory.txt` 里与你负责的 mode 有关的小节（VOFA 填充函数体、extern 列表、
   宏列表），先建立"有哪些真实符号"的清单。
3. 读你负责的 `foc_<NN>_*.c` / `.h` 全文（文件不大，几百行），确认每个符号的语义。
4. 用 grep 在 `D:\WS_L_re\md_record\` 里搜该 mode 号 / 文件名关键词 / 关键符号名，找有没有实测结论
   （定稿参数、实测波形、已知问题）。有就写进 `notes` 或 `(*)/([!])` 注里。
5. 写 JSON 文件。写完后**自己回读一遍**，逐条核对：符号名是否与源码逐字一致、通道数是否等于
   `return` 的值、通道顺序是否一致、有没有混入宽度不确定字符。
6. 返回一段简短总结（不要贴整个 JSON）：mode 号、填充函数名与通道数、watch/obs 各几条、
   你**没写**进去的内容及原因（例如找不到符号）。

## 注意
- 卡片的读者是这块板子的作者，他要的是"这个 mode 怎么用、每个 Watch 变量和 VOFA 通道是什么、
  怎么判断跑对了、哪里容易踩坑"。不要写空话（如"用于调试"），要写具体判据和阈值。
- 宁可少写几条，也不要写不确定的内容。查不到的信息，在总结里说明，不要编。

# SiFli-SDK — Skaiwalk monorepo submodule

`C:\skaiwalk` monorepo 的 submodule（SiFli 廠商 SDK 的 fork）。sub-repo session 不繼承 root 的 slash commands / skills，需手動載入。

## UI 任務前必讀
Watch UI 設計 / 實作前，先讀 monorepo 的統一設計規範：
`C:\skaiwalk\.claude\skills\Skaiwalk_UI\SKILL.md`（design tokens、component spec、anti-patterns、a11y 基線、§9 自我迭代協議）。
遇使用者修正 UI 規則，回該檔走 §9 patch 並補 §10。

## 本 repo 專屬 skill（手動讀）
- `.claude/skills/dualcore-ipc/SKILL.md` — HCPU/LCPU data_service IPC pattern（跨核傳資料前必讀）
- `.claude/skills/pc-sim-dev/SKILL.md` — Win32 LVGL PC 模擬器開發 / 除錯流程

## 紀律
改完 code 停在總結、列出動了哪些檔；編譯 / 刷機由使用者執行，除非明講要你跑。完整規範見 monorepo root `C:\skaiwalk\CLAUDE.md`。

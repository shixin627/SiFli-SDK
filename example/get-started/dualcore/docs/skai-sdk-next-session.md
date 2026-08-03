# Skai SDK — handoff

Everything decided is in [ADR-0019](adr/0019-skai-sdk-two-tier-and-self-signing.md)
(decisions 1–13, plus every measured number). Phasing and status are in
[skai-sdk-plan.md](skai-sdk-plan.md). This file is only what the next session
needs to start moving.

## State

Phase 0/1/2 done. Phase 3 partly done: QuickJS runs sandboxed on the PC
simulator, capabilities are auto-injected from the dispatch table, and a JS app
draws and reacts to taps. `PKG_USING_QUICKJS` is on in `pc_hcpu/proj.conf`
**only** — the watch build stays green and the 95 KB flash shortfall stays a
separate deployment question.

```
project\hcpu\_pc_build.cmd -j8
project\hcpu\_dev_test.cmd -nobuild -script _skai_js.txt      # sandbox gates
project\hcpu\_dev_test.cmd -nobuild -script _skai_test.txt    # capability layer
project\hcpu\_dev_test.cmd -nobuild -script _skai_gate1.txt -screenshot x.png
python tools\sdk\gen_dispatch.py --selftest && python tools\sdk\gen_dispatch.py
```

Both suites are green. `gen_dispatch.py` also emits `skai-api.html`, the public
API reference — every `ui.*` capability added for the calculator shows up there
automatically, so there is no separate doc step to remember.

**Do not paste internal wording into a capability's doc comment.** Those
comments become the public reference. It already caught three leaks: backing
globals, `ponytail:` maintainer notes, and a line telling third parties about
"the first sync from LCPU". The generator strips `ponytail:` paragraphs and
drops module-level blurbs; everything else you write is published verbatim.

## Resolved: the allocation-storm crash, and the hole under it

`skai_js_test` reaches its summary again — `PASS (0 failures)`, storms A and B
both entering *and* exiting. The suite was not weakened to get there.

**The memory quota was never a byte quota.** QuickJS charges every allocation a
flat 8 bytes (`js_def_malloc_usable_size()` returns 0 for `_WIN32` *and* for the
`#else` fallback armclang takes, quickjs.c:1696), so a 192 KB limit really meant
"24576 allocations". The simulator's own memory dump says it out loud: `8.0 per
block`. **This was live on the watch too** — guarantee 1 of the sandbox was not
in force on hardware.

The crash followed from it: with bytes uncounted, an app drains the heap QuickJS
allocates from before the quota notices, allocations then fail for real, and
QuickJS's out-of-memory path dies in that state — `build_backtrace →
JS_DefineProperty → find_own_property`, on a shape a failed allocation left
behind.

Fixed in `skai_js.c` with our own `JSMallocFunctions` (8-byte header carries the
requested size; refusal latched at the source; 25 % reserve for the error path,
closed again on the next entry). Details and the amended decision are in
[ADR-0019 §11](adr/0019-skai-sdk-two-tier-and-self-signing.md). Two things came
free: OOM no longer has to be inferred from an unreadable exception, and an app
throwing an object whose `toString()` throws is no longer mislabelled a memory
failure.

Both guesses in the previous handoff were wrong, and cost a rebuild each:

1. **`CONFIG_STACK_CHECK` under MSVC** — defining it only for non-MSVC is
   correct and has been kept (`rtconfig_project.h`), but with the guard compiled
   out entirely the storm still died. Not the cause.
2. **The watchdog firing mid-allocation** — with the deadline at 60 s the storm
   still died. Not the cause.

### How to find the next silent simulator death

The sim dies with no console output, which is what made this expensive. There is
no debugger installed, but nothing more is needed:

```powershell
# fault offset (RVA) of the last crash
Get-EventLog -LogName Application -Newest 30 |
  Where-Object { $_.Source -eq "Application Error" -and $_.Message -match "main.exe" } |
  Select-Object -First 1 -ExpandProperty Message
# symbol + line, straight from main.pdb
& "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.44.35207\bin\Hostx64\x64\llvm-symbolizer.exe" `
  --obj=build_pc_hcpu\main.exe --relative-address 0x000a2d25
```

For the whole call chain rather than one frame, a `SetUnhandledExceptionFilter`
+ dbghelp `StackWalk64` handler in a temporary file gets it in ~60 lines
(`#pragma comment(lib, "dbghelp.lib")`; write to a **file** — `printf` from the
handler does not reach the captured console log).

## Then: finish app_calculator

`src/hcpu/gui_apps/calculator/app_calculator.c`, 416 lines, and the smallest
honest target — simple enough to actually match, real enough to expose gaps.

It creates **four** widgets:

| widget | role |
|---|---|
| `lv_textarea_create` | the number display |
| `lv_btnmatrix_create` | the whole keypad, as ONE widget |
| `lv_line_create` | divider |
| `lv_btn_create` | back |

Styling used: `bg_color`, `bg_opa`, `text_color`, `text_font`, `radius`,
`set_size`, `pad_all`, `text_align`, `line_width`, `line_color`.
Two `lv_obj_add_event_cb`.

### What `skai_ui` is missing for it

- **keypad** — `lv_btnmatrix` is a gift for a curated API: one widget, a list of
  labels, and the click reports which label. Map it to something like
  `ui.keypad("7 8 9 /\n4 5 6 *\n...")` returning one id, with the existing
  `ui.on_click` handing back the button text. Safer than 20 slots and closer to
  what the C app actually does.
- **display field** — a right-aligned, large-font label covers it; a real
  textarea is not needed since the app never takes keyboard input.
- **divider** — a 1px styled container, not a new widget kind.
- **styling** — font size and background colour are the two genuine gaps.
  `ui.set_color` only does text today.

### Success criterion

Not "the API has a keypad". Screenshot the C app and the JS app and iterate
until a person cannot tell them apart. That is what finds the real gaps, and it
is the loop worth automating.

Note the standing tension before widening the API: pixel-identity with
hand-tuned C screens pulls toward exposing all of LVGL styling, which is what
ADR-0019 decision 9 forbids for untrusted code. Where a gap can be closed by a
**higher-level** primitive (keypad, not 20 buttons), close it that way.

## Also still open

- **Install-time signature verification** — the fourth sandbox piece
  (mbedtls ECDSA P-256 + SHA-256 + TOFU keyid). Not started.
- **Microphone in-use indicator** — `lv_layer_sys()` + capture refcount. Its own
  work block; blocks opening T2-mic.
- **Recursion guard on hardware** — the simulator cannot test it (QuickJS's MSVC
  `js_get_stack_pointer` returns a code address, so `rtconfig_project.h` now
  enables `CONFIG_STACK_CHECK` for non-MSVC only). The armclang path is correct;
  verify on device.
- **The byte ceiling on hardware** — the new accounting is verified on the
  simulator only. On the watch the JS heap is the PSRAM memheap
  (`QUICKJS_USING_PSRAM`), so confirm there that a storm now reports
  `memory-quota` well before that heap is drained.
- **95 KB flash shortfall** — QuickJS does not fit the 2.5 MB `main` partition.
  Growing it cannot ship as an OTA.
- **capability-registry.json → phone** — the generated registry should feed
  SkaiLink's AI prompt and validator, so a new capability becomes usable by the
  generator without a phone release. Last link in the chain, not wired.

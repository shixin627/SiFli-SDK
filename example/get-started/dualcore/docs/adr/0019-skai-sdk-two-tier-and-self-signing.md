# ADR-0019 — Skai SDK: two-tier API, self-signed packages, generated dispatch table

- **Status**: accepted (2026-08-03), Phase 0 landed
- **Scope**: freezes only what is a one-way door. Everything else stays movable.
- **Plan**: [`docs/skai-sdk-plan.md`](../skai-sdk-plan.md) — rationale, hardware
  survey, phasing. This ADR records the decisions; the plan stays a living doc.

## Context

`gui_apps/skaiapp/` hardcodes third-party capability in two enums (8 widgets,
9 binds, 5 actions). Adding one capability costs four file edits plus a
firmware release, which is precisely why the platform cannot be opened.
Capability is not the problem — every backend already exists in `bloc/`,
`client/`, `service/`. The boundary is.

## Decision

### 1. Two tiers, split on one fact: internal staff can flash, external developers cannot

| | internal C API | external script API |
|---|---|---|
| update unit | whole firmware, same commit as its callers | app only; firmware is a variable they do not control |
| compatibility | **API, not ABI** — refactor freely, the compiler catches breaks | **frozen** — an installed app must not break on new firmware |
| sandbox / quota / signing | not needed | **required** |

Everything else follows from that row. The internal layer therefore gets no
opaque handles, no versioned structs, no compat shims — spending effort there
buys nothing and delays the parts that are genuinely irreversible.

### 2. The external API is a projection of the internal one, never a second hand-written copy

`SKAI_EXPORT()` annotations on C headers are the single source of truth.
`tools/sdk/gen_dispatch.py` projects them into the firmware dispatch table,
the phone-side capability registry, and a developer `.d.ts`. Two hand-kept
API surfaces always drift, and the external one always loses.

### 3. Packages are self-signed; the permission model is therefore the only security boundary

Anyone can publish. Signing keeps exactly three properties — integrity,
publisher continuity (TOFU), attributability — and confers **no** authorization.
The consequence, which is the important half of this decision: permissions,
quotas and the sandbox stop being polish and become the only thing standing
between a user and a bad app. Phase 3's malicious-app acceptance test is a
gate, not a bonus.

eFuse RoT is **not** used here; that root belongs to secure boot, for firmware.
A self-signed app is its own root, with its public key in the manifest.

### 4. Six manifest fields are the only truly irreversible artifact

Frozen now because external developers cannot flash, so a published format is
carried forever: version negotiation, capability declaration, public key,
signature, `keyid` as part of the storage primary key, and a revocation list
format.

Two consequences that are easy to get wrong and expensive to retrofit:

- **`(keyid, app_id)` is the primary key**, not `app_id`. Under self-signing
  two developers will eventually claim the same id, and a bare-`app_id` store
  lets one silently overwrite the other.
- **The blocklist format ships before the first app does.** Who supplies
  entries is still open; the format cannot be. Self-signing without a
  revocation path means no way to stop the bleeding after an incident.

### 5. Signing envelope, not an extended package

`skaiapp-manifest.schema.json` wraps a payload; it never contains it. Putting
signature fields inside the signed object forces JSON canonicalization, which
is a well-known source of signature-bypass bugs. Instead the signature covers
a delimited byte string (`signedInput`) and binds the payload only by its
SHA-256, so the firmware verifier needs no JSON canonicalizer.

**Capabilities are inside the digest.** Otherwise an attacker could widen an
app's permissions without invalidating the signature — which would defeat
decision 3 entirely. `skai` is deliberately outside it, so the same payload
can be re-issued for a wider firmware range.

Legacy bare packages stay valid: firmware sniffs the top-level key
(`skai` = signed manifest v1, `skaiapp` = legacy v0). Packages already pushed
to users' watches must not break.

### 6. Capability names are an open pattern, not a closed enum

A closed enum in the schema would reintroduce the exact coupling this SDK
exists to remove — every new capability would need a schema release. The
schema constrains the *shape*; the generated registry constrains the *set*, so
developers still fail at build time rather than on the wrist.

## Phase 0 de-risk probes — measured, and they moved the risk

Both were measured against this repo by building it, not estimated. The second
one **inverted the plan's top risk**.

**QuickJS/LVGL binding compatibility — clear, no shim.** Project LVGL is
**v8.3.1**; `external/quickjs/lvgl/` is a v8 binding (`lvgl_v8_qjs.c`, selected
by its SConscript). Of 119 `lv_*` symbols it references, 54 are absent from
LVGL core — but all of them resolve: 40 into `middleware/lvgl/lvsf/` (SiFli's
extension widgets, **already linked into this firmware** — 4648 references in
`main.map`) and 14 are defined inside the binding itself. All nine QuickJS
translation units compile clean against 8.3.1. The weeks-of-shim risk is gone.

**The blocker is flash, not SRAM.** Enabling
`PKG_USING_QUICKJS` + `QUICKJS_LVGL` + `QUICKJS_USING_PSRAM` compiles but
**fails to link**: `L6407E, aggregate size 0x17be8` — **95 KB short**.

Measured with `fromelf --text -z` on the objects:

| | code + RO (flash) | static SRAM |
|---|---|---|
| `quickjs.o` | 241.4 KB | 4 B |
| `libunicode.o` | 41.4 KB | 0 |
| `libregexp.o` + `cutils.o` + `quickjs-libc.o` + `qjs.o` | 18.2 KB | 4 B |
| LVGL binding (3 TUs) | 11.7 KB | 0.8 KB |
| **total** | **312.7 KB** | **0.8 KB** |

Against `ER_IROM1` headroom of **264 KB** (`Size 0x23dff4` of `Max 0x280000`,
the 2.5 MB `main` partition). The remaining ~46 KB of the shortfall is libc/libm
that QuickJS drags in (float formatting, `usenofp`, `use_no_semi`).

So the plan's stated top risk is backwards. **HCPU SRAM is not the constraint**:
static SRAM cost is 0.8 KB against a 357 KB heap (image ends `0x2006AA38`, heap
runs to `HCPU_RAM_DATA_END` `0x200C3C00`, per `main.map` + `bsp_board.h`). The
"chronically full" comment in `skaiapp_pkg.h` is about runtime pressure, not
this budget. The JS heap goes to PSRAM regardless.

**The 2.5 MB `main` partition is what Phase 3 has to solve.** Three routes, not
yet chosen — this is a Phase 3 design item, and it is a partition/bootloader
change, so it cannot ship as an OTA:

1. Grow `main` in `ptab.json`. The table maps ~106 MB of 960 MB flash, so flash
   itself is free; but `main` executes from PSRAM (`app_exec`, 2.5 MB of 8 MB),
   so the PSRAM exec region grows with it.
2. Drop `libunicode.o` (41 KB) by building QuickJS without full Unicode tables.
   Needs a Kconfig knob in `external/`, which is don't-touch.
3. Reclaim 95 KB from the existing image.

Route 1 is the only one that scales; 2 and 3 only buy headroom once.

### 7. `SKAI_NO_DATA` is `INT32_MIN`, not "any negative value"

Phase 1 used "negative means no reading". Phase 2 added `weather.temp` and that
convention immediately forbade -5 C. A sentinel that bans negative data would
have been an invisible ceiling on every future capability — temperature,
altitude, deltas, offsets — so it became a single reserved value before
anything shipped. The negative returns from the bounded-string functions are
error codes, not data, and are unaffected.

### 8. Declarative binds resolve by name; only package-local ones keep an enum

`skaiapp`'s bind enum conflated two different things. Watch state (time,
battery, heart rate, ...) is now a single `SKAIAPP_BIND_CAP` plus a dispatch
index resolved by name, so a new capability needs no parser or renderer edit.
Package-local references (`timer:`/`reminder:`/`memo:`) point at objects inside
the package, which the dispatch table knows nothing about, and keep their own
enum values and slot index.

Display format lives in the dispatch table alongside the capability (the
optional fourth `SKAI_EXPORT` argument), because a units table maintained
separately in the renderer is exactly the drift the generator exists to
prevent.

Two things make the old packages keep working:

- Packages are stored as **raw JSON** and re-parsed on every load, so no
  installed package holds these enum values. Changing them is free.
- A **frozen** five-entry compat table maps the v0 bind keywords
  (`time`/`date`/`battery`/`hr`/`steps`) onto capability names, and carries the
  one legacy default that cannot be derived — `steps` gauges assume a goal of
  8000 when the package omits `max`. It must never grow: anything new uses a
  capability name directly.

### 9. The vendored raw LVGL binding is not the external drawing API

`external/quickjs/lvgl/` exposes roughly 200 raw `lv_obj_*` calls straight to
script. For an internal scripting toy that is fine; for untrusted third-party
code it hands attacker-controlled arguments directly to LVGL and bypasses the
capability model entirely — the two things decision 3 says are the only
security boundary left. `QUICKJS_LVGL` therefore stays **off**, and third-party
drawing goes through curated `skai_ui.*` capabilities projected from the
dispatch table like everything else.

(The binding also fails to build on the simulator: it references `lv_ezipa_*`
and `lv_gif_*` hardware decoders the PC config does not include. That is a
symptom, not the reason.)

### 10. Phase 3 develops on the simulator, which sidesteps the flash blocker

QuickJS does not fit the 2.5 MB `main` partition (95 KB short, see the Phase 0
probe). All four Phase 3 acceptance gates are defined on the PC simulator,
which has no such partition, so `PKG_USING_QUICKJS` is enabled in
`pc_hcpu/proj.conf` **only** — the watch build stays green and the partition
decision stays a separate, deployment-shaped question rather than a blocker on
the sandbox work.

Measured while enabling it: QuickJS core (`quickjs.c`, `libregexp`,
`libunicode`, `cutils`) compiles clean under MSVC. Only the LVGL binding did
not, for the reason above.

### 11. Quotas are QuickJS's, not ours — except the memory ceiling, which cannot be

`JS_SetInterruptHandler` already polls between bytecodes, so the watchdog is
QuickJS's and a supervisor thread would only be more code. `JS_SetMaxStackSize`
matters just as much and is easy to forget: without it, runaway recursion
overflows the RTOS thread stack and takes the watch down instead of throwing.

Failure causes are kept **distinct** — timeout, memory, permission denial,
exception — because "the app stopped" is not something an external developer
can act on, and they cannot attach a debugger.

**The memory ceiling had to move to us.** Amended during Phase 3 testing; the
original decision was `JS_SetMemoryLimit`, and it does not work here. QuickJS
accounts every allocation as

```c
malloc_size += js_def_malloc_usable_size(ptr) + MALLOC_OVERHEAD;
```

and `js_def_malloc_usable_size()` returns 0 both for `_WIN32` and for the
`#else` fallback armclang takes (quickjs.c:1696). Every block therefore costs a
flat 8 bytes of quota whatever its real size: a 192 KB limit is really "24576
allocations", and an app can hold far more memory than its manifest declared.
Measured, not deduced — the simulator's own `JS_ComputeMemoryUsage` dump reads
`8.0 per block`. **This affected the watch, not only the simulator.**

That is also what crashed the simulator. With bytes uncounted an app drains the
heap QuickJS actually allocates from before the quota notices; allocations then
start failing for real, and QuickJS's out-of-memory path is not robust in that
state — it died inside `build_backtrace → JS_DefineProperty →
find_own_property`, on a shape a failed allocation had left behind. A ceiling
that trips while the heap still has room keeps that path out of trouble.

`skai_js.c` therefore supplies its own `JSMallocFunctions`. An 8-byte header
per block carries the requested size, which is what makes byte accounting
possible with no usable-size primitive to ask. Allocation itself is still the
platform's, through the same `cutils.h` switch QuickJS uses, so the JS heap
still lives in PSRAM on the watch — only the counting changed. Two things fall
out of owning it:

- a refusal is **latched at the source**, so classification no longer has to
  infer the cause;
- past the ceiling a **25 % reserve** opens, for the error path only, and
  closes again on the next entry. QuickJS has to allocate to build its
  out-of-memory exception, and the reserve is why that can no longer fail.

**Detecting out-of-memory was not what it looked like**, and the reasoning is
kept because it is why the latch replaced it. QuickJS reports OOM by throwing
`InternalError("out of memory")`, but building that Error object needs
allocations, and at OOM there are none — so the object is never constructed
(`JS_ThrowOutOfMemory`, quickjs.c). Everything downstream sees an exception with
nothing to read. Three approaches fail for that reason:

- matching the message text — there is no message;
- `JS_ComputeMemoryUsage()` afterwards — the failed allocation is already freed,
  so usage sits back under the limit;
- raising the limit at classification time — too late, the Error was never built.

An unreadable exception was therefore the signal, with a documented ceiling: an
app throwing an object whose `toString()` throws landed in the same branch and
was mislabelled a memory failure. The latch removes both the inference and the
mislabel; the message-text match stays only for an OOM QuickJS raises without
an allocation of ours failing.

**The vendored QuickJS ships with its stack guard disabled on RT-Thread**, and
that hole is only visible by testing for it:

```c
#if !defined(EMSCRIPTEN) && !defined(BSP_USING_RTTHREAD)
#define CONFIG_STACK_CHECK
#endif                                    /* external/quickjs/quickjs.c:81 */
```

This project defines `BSP_USING_RTTHREAD`, so `js_check_stack_overflow()`
compiles down to `return FALSE` and `JS_SetMaxStackSize()` stores a number
nobody reads. `function f(){return f();} f();` in a third-party app then runs
off the native stack — process death on the simulator, hard fault on the watch.
The sandbox test caught it because it tries the attack rather than asserting the
API was called.

Fixed by defining `CONFIG_STACK_CHECK` in `project/hcpu/rtconfig_project.h`
under `PKG_USING_QUICKJS`: `rtconfig.h` is force-included ahead of that `#if`,
so the macro is already set when it runs and `external/` stays untouched.

**Non-MSVC only.** With the guard enabled, QuickJS's MSVC `js_get_stack_pointer()`
returns `_ReturnAddress()` — a *code* address, not a stack address
(quickjs.c:1603) — so `js_check_stack_overflow()` compares it against a limit
derived from a real stack address and can fire anywhere. The armclang path uses
`__builtin_frame_address(0)` and is correct, which is the path that matters;
the simulator cannot test the guard either way.

A memory limit and a watchdog are not enough on their own. Anything that runs
untrusted code needs all three, and the third is the one that is silently
absent.

### 12. A JS app is a GUI app, and that is what makes drawing legal

`skai_ui` refuses every call from a thread other than LVGL's, so the
interpreter has to run where the widgets live. `app_skaijs` hosts it: the app
framework's mailbox already delivers `on_start` on the LVGL thread. Widgets are
addressed by slot id, never by pointer, so a hostile app cannot turn a number
into a wild write — same reasoning as the timer slot table.

Gate 1 passes on the simulator: a JS app reads the heart rate, computes a
percentage, draws an arc, reads the clock, and vibrates, entirely through
dispatch-table capabilities its manifest declared.

### 13. Reproducing the built-in apps needs an event loop, not more widgets

Measured across `gui_apps/`: `lv_obj_create` 291, `lv_label_create` 249,
`lv_img_create` 126, `lv_btn_create` 63 — four kinds cover roughly 90% of all
widget creation. But `lv_obj_add_event_cb` appears **266** times, plus
`lv_anim_start` 94, `lv_obj_scroll*` 67 and `lv_timer_create` 49.

So the gap is not the widget vocabulary; it is that nothing interactive can
exist yet. A callback has no projection in the dispatch table's type
vocabulary, which is why `skai_timer`'s callbacks are unexported too. Both need
the same missing machinery.

**This changes the sandbox, and that is the part to get right first.** Today
`skai_js_eval` creates a runtime, runs, and frees it, so every quota is
per-run. An interactive app needs its context to live from `on_start` to
`on_stop`, which means:

- the memory quota becomes a **session** budget — an app may creep up to the
  cap over minutes rather than blowing it at once, so the reporting has to
  distinguish "leaking" from "exceeded";
- the watchdog must be **re-armed per callback**, not one deadline for the
  whole app, or a long-lived app trips it just by staying open;
- the JS heap occupies PSRAM for as long as the app is on screen.

Deciding those semantics is a prerequisite for the widgets, not a follow-up.

## Consequences

- Adding a capability becomes one annotated declaration; the dispatch table,
  the phone validator's registry and the developer types all follow.
- Phase 1 loses roughly a week by not freezing the internal layer.
- The mic in-use indicator (`lv_layer_sys()`, bound to capture refcount) and
  the app-scoped debug pipeline become load-bearing, not optional — both are
  Phase 3 hooks because retrofitting either is painful.
- Getting the six manifest fields wrong is the one mistake this project cannot
  walk back.

## Open

1. **Blocklist source** — Skaiwalk backend, user-initiated, or both. Format is
   frozen; source is not.
2. **Background microphone** — currently foreground-only for self-signed apps,
   because the indicator is only visible while the screen is on. A legitimate
   background case (e.g. third-party snore detection) would need a separate
   mechanism such as an always-on-display indicator.
3. **Haptics tier** — the tier table in the plan does not place haptics.
   `skai_haptic.h` assumes T1 (Pebble grants vibes to every app; the abuse
   ceiling is annoyance, not data). Confirm before the first external app
   ships: tightening a tier later is a breaking change.

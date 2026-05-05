# MPP vs. dav1d 1.5.3: parser-output disagreements on `reference_select`, `skip_mode_present`, and `loop_filter.mode_ref_delta_enabled`

This document summarizes — at the AV1 spec level, without copying or paraphrasing
algorithm internals — why MPP's AV1 parser and dav1d 1.5.3's AV1 parser can produce
different DXVA / API outputs for three frame-header fields, even when both are
fed identical bitstream bytes.

The investigation deliberately stays at the level of:

- "what bit, at what spec position, is read"
- "what gates the read"
- "what state survives across frames"

so that a clean-room implementation can be written from this document plus the
public AV1 spec, without referring to MPP source.

## 0. Source map — where to look (citations only)

The user-supplied evidence cited `av1d_codec.c:1212-1295` and
`hal_av1d_vdpu.c:2092-2093`. **Those filenames/lines are from a different MPP
checkout.** In the copy of MPP under study here, the relevant code lives in:

| Concern                                | File                               | Function                                    | Line range  |
|----------------------------------------|------------------------------------|---------------------------------------------|-------------|
| `reference_select` parse               | `av1d_cbs.c`                       | `mpp_av1_frame_reference_mode`              | 1450–1462   |
| `skip_mode_present` parse + gating     | `av1d_cbs.c`                       | `mpp_av1_skip_mode_params`                  | 1464–1546   |
| `loop_filter_delta_enabled` parse      | `av1d_cbs.c`                       | `mpp_av1_loop_filter_params`                | 1276–1359   |
| Uncompressed header order              | `av1d_cbs.c`                       | `mpp_av1_uncompressed_header`               | 1732–2144   |
| Frame-OBU dispatch / per-OBU calloc    | `av1d_cbs.c`                       | `mpp_av1_alloc_unit_content`, `mpp_av1_read_unit` | 2770–2900 |
| DXVA fill (frame-header → pic-params)  | `av1d_parser2_syntax.c`            | `av1d_fill_picparams`                       | 24–281      |
| `AV1Context` (parser-persistent state) | `av1d_parser.h`                    | struct AV1Context                            | ~120–152    |
| Per-stored-ref persisted state         | `av1d_cbs.h` / `av1d_cbs.c`        | `AV1ReferenceFrameState`, update at 2115–2141 |             |

The dav1d-side citations are all in `dav1d/src/obu.c` (1.5.3).

## 1. Bitstream order is identical between the two parsers

Both parsers consume the uncompressed frame-header bits in the order specified
by AV1 §5.9.1 *uncompressed_header()*. Concretely, after `tx_mode_select`:

1. *frame_reference_mode()* (§5.9.20) — 1 bit, gated on inter/switch frame.
2. *skip_mode_params()* (§5.9.21) — 1 bit, gated on `SkipModeAllowed`.
3. *frame_reference_mode* itself precedes *skip_mode_params* in the spec, and
   both parsers honor that order.

In MPP this is `av1d_cbs.c:2087` (frame_reference_mode call) immediately followed
by `av1d_cbs.c:2090` (skip_mode_params call). In dav1d this is `obu.c:934-935`
followed by `obu.c:940-994`. The bit positions consumed are identical. **There
is no bit-position drift — neither parser reads an extra optional element that
the other skips.**

The same applies to *loop_filter_params()* (§5.9.11): MPP at `av1d_cbs.c:2075`
calls `mpp_av1_loop_filter_params`, dav1d at `obu.c:834-872` inlines it. The
relevant single bit `loop_filter_delta_enabled` is at the same spec position
(`av1d_cbs.c:1317` ↔ `obu.c:858`) and gated by the same condition
(`coded_lossless || allow_intrabc`).

So **all three fields are parsed from the same bits in the same bitstream
locations** in both implementations. The differences observed at the BSP
register level cannot come from a bitstream-offset divergence.

## 2. The actual divergences are in (a) cross-frame state and (b) which DXVA
       slot is filled when a per-frame value isn't parsed

### 2.1 Question 1 (`sw_skip_ref1` / `pp->skip_ref0` / `pp->skip_ref1`)

**Spec-level summary of what MPP reads / computes:**

- `skip_mode_present` (spec §5.9.21) — 1 bit, only when `SkipModeAllowed == 1`,
  otherwise inferred 0. MPP: `av1d_cbs.c:1540-1543`.
- `SkipModeAllowed` derivation (spec §5.9.22) — searches for forward/backward
  reference frames by `OrderHint` distance. MPP: `av1d_cbs.c:1471-1538`.
- The forward/backward indices that the spec calls `SkipModeFrame[ 0 ]` and
  `SkipModeFrame[ 1 ]` are written to `ctx->skip_ref0` / `ctx->skip_ref1` and
  carried through to DXVA as `pp->skip_ref0 / pp->skip_ref1` (offset by +1 to
  match the BSP "0 = LAST default, n = ref-position+1" convention).

**Where the parsers disagree (state lifecycle, NOT bits):**

`ctx->skip_ref0` and `ctx->skip_ref1` live in `AV1Context` (the persistent
parser context — see `av1d_parser.h:146-147`), **not** in the per-frame
`AV1RawFrameHeader`. They are written **only** at:

- `av1d_cbs.c:1509-1510` — when both forward and backward refs exist.
- `av1d_cbs.c:1532-1533` — when only forward refs exist (two-forward case).

They are **never reset at frame entry**. (Per-OBU `AV1RawOBU` is
`mpp_calloc`-zeroed at `av1d_cbs.c:2774`, but `AV1Context::skip_ref0/1` lie
outside that allocation.) The DXVA fill at `av1d_parser2_syntax.c:270-271`
copies `h->skip_ref0/1` into `pp->skip_ref0/1` **unconditionally** — there is no
gate on `frame_header->skip_mode_present` nor on `frame_header->reference_select`.

dav1d, in contrast, allocates a fresh `Dav1dFrameHeader` per frame from a pool
and `memset`s it to zero at `obu.c:1280`. Its analogous fields
`hdr->skip_mode_refs[0/1]` / `hdr->skip_mode_allowed` therefore start at 0 each
frame, and are only written inside the `switchable_comp_refs && IS_INTER_OR_SWITCH
&& seqhdr->order_hint` block at `obu.c:940-991`.

**Net effect on the captured BSP register `sw_skip_ref1 = 0x14` (= 5 = BWDREF
position+1) on a frame where the regbuilder (driven by dav1d) emitted 0x04
(= 1 = LAST default):**

- The MPP-driven side is reporting a stale `skip_ref1` from an *earlier* frame
  in which `SkipModeAllowed` evaluated to 1 with `backward_idx == 4` (the
  BWDREF slot in the parser's per-frame ref ordering — index 4 of seven, i.e.
  `AV1_REF_FRAME_BWDREF - AV1_REF_FRAME_LAST = 4`). On the current frame
  (whatever its `skip_mode_present` value), MPP did not overwrite
  `skip_ref0/1`, so the prior value bled through into the DXVA params and on to
  the BSP HAL.
- The dav1d-driven side correctly reports the per-frame value, which on a
  low-delay (no true B) stream is 0 because `hdr->skip_mode_allowed` stayed 0.

**This means MPP's behavior here is *not* a different bitstream interpretation —
it is per-frame-stale state being copied unconditionally into the
hardware-facing params struct.** The HAL's `(skip_ref{0,1} != 0) ? skip_ref{0,1}
: 1` clamp then turns "stale 5" into the captured `sw_skip_ref1 = 5`.

The user's premise — "MPP's parser writes ctx->skip_ref0/1 = 0 at top of
function" — is **not true in this MPP copy**. There is no such reset; the fields
are only ever written in the success branches at lines 1509-1510 / 1532-1533.

### 2.2 Question 2 (`sw_comp_pred_mode` / `pp->coding.reference_mode`)

**Spec-level summary:** The DXVA field `reference_mode` is filled **directly
and verbatim** from the parsed `reference_select` bit:

```
pp->coding.reference_mode = frame_header->reference_select;  // av1d_parser2_syntax.c:95
```

`reference_select` is a 1-bit `f(1)` read in spec §5.9.20, gated on
`!FrameIsIntra` (i.e. `frame_type != KEY && != INTRA_ONLY`). MPP:
`av1d_cbs.c:1455-1459`. dav1d's `hdr->switchable_comp_refs` is the same bit at
the same position, gated on `IS_INTER_OR_SWITCH(hdr)` (`obu.c:934-935`). The
two gating expressions are equivalent (KEY = 0, INTER = 1, INTRA_ONLY = 2,
SWITCH = 3 in both implementations).

**Therefore MPP and dav1d *cannot* parse different values for
`reference_select` ↔ `switchable_comp_refs` from the same bitstream**, *unless*
they have already diverged on `frame_type`, which would be a much larger
disagreement than just one bit.

If the user's regbuilder is observing a 12-frame divergence, the likely sources
(in order of probability) are:

1. The regbuilder is reading a dav1d field that is *not* the raw
   `switchable_comp_refs` bit. In dav1d 1.5.3, `hdr->switchable_comp_refs` is
   the unmodified parsed bit. There is no field called "reference_mode" in
   `Dav1dFrameHeader` that mirrors MPP's DXVA `reference_mode`. If the
   regbuilder is computing `reference_mode` from, e.g.,
   `hdr->switchable_comp_refs && (something else)`, that is the divergence
   point and it is on the *consumer* side, not the parser side.
2. The HAL formula `(reference_mode != 0) ? 2 : 0` collapses any non-zero
   reference_mode value to 2. MPP's DXVA `reference_mode` is just the 1-bit
   parsed value (0 or 1), so it can only ever produce `0` or `2` at the
   register. If the captures show `2` where the regbuilder emits `0`, that
   matches the case where the bitstream's `reference_select == 1` but the
   regbuilder is using a synthesized "is reference mode actually used?" signal
   instead of the raw bit.

**Recommended clean-room mapping:** treat `pp->coding.reference_mode` as
"raw value of the `reference_select` syntax element from §5.9.20, with the
intra-frame infer-to-0 rule applied." Do not derive it from any post-parse
inference about whether compound prediction is actually used.

### 2.3 Question 3 (`sw_filt_ref_adj_*` / `pp->loop_filter.mode_ref_delta_enabled`)

**Spec-level summary:** `loop_filter_delta_enabled` (DXVA name
`mode_ref_delta_enabled`) is parsed as a 1-bit `f(1)` element of
*loop_filter_params()* (spec §5.9.11), gated on
`!CodedLossless && !allow_intrabc`. MPP: `av1d_cbs.c:1317`. dav1d:
`obu.c:858`.

When the gate is hit (`CodedLossless || allow_intrabc`), spec §5.9.11
specifies a "skip the rest of loop_filter_params" early-return. The two
implementations behave **differently** in that early-return path with respect
to the *value of `mode_ref_delta_enabled`* surfaced to the consumer:

- **MPP** (av1d_cbs.c:1284-1298): infers `loop_filter_level[0..1]` to 0 and
  `loop_filter_ref_deltas[]`/`loop_filter_mode_deltas[]` to the AV1 default
  set, then returns. **It does not write `loop_filter_delta_enabled`**, so it
  remains whatever the per-OBU calloc left it: **0**. The DXVA fill at
  `av1d_parser2_syntax.c:166` then publishes `mode_ref_delta_enabled = 0`.
- **dav1d** (obu.c:835-838): in the same lossless/intrabc branch, sets
  `hdr->loopfilter.mode_ref_delta_enabled = 1` and points
  `loopfilter.mode_ref_deltas` at the default mode/ref deltas table.

**On lossy, non-intrabc inter frames, both parsers read the same single bit
from the same bitstream position, with the same default-inheritance behavior
(default deltas if `primary_ref_frame == PRIMARY_REF_NONE`, else inherited
from the primary ref's stored `loop_filter_ref_deltas` / `_mode_deltas`).**

So the only path where the parsed *value* of `mode_ref_delta_enabled` itself
diverges is `coded_lossless || allow_intrabc`. For a generic B-pyramid stream
with normal lossy inter frames, this path is rare-to-impossible, and
`mode_ref_delta_enabled` per-frame should match. If the user sees 24+ frames
where the BSP value disagrees with the regbuilder's value, the most likely
causes are:

1. **Stale-state inheritance via `primary_ref_frame` slot mismatch.**
   Both parsers, when `delta_update == 0`, inherit the actual `ref_delta[]` /
   `mode_delta[]` *values* from the primary reference slot's stored copy:
   - MPP: `ctx->ref_s[ ref_frame_idx[ primary_ref_frame ] ].loop_filter_ref_deltas`
     (av1d_cbs.c:1325-1329).
   - dav1d: `c->refs[ refidx[ primary_ref_frame ] ].p.p.frame_hdr->
     loopfilter.mode_ref_deltas` (obu.c:850-857).
   If MPP's `ctx->ref_s[i].loop_filter_ref_deltas` was *last written* by a
   different frame than dav1d's `c->refs[i].p.p.frame_hdr` was associated
   with — for example because of a dropped/replaced frame, an
   `error_resilient_mode` path, or a `show_existing_frame` event — the two
   slots can hold different deltas, and the `mode_ref_delta_enabled` flag
   applied to those mismatched deltas at the HAL produces the observed
   register divergence.
2. **Lossless / intrabc transient frames** where MPP emits 0 and dav1d emits
   1, see above.
3. The HAL gate. `hal_av1d_vdpu.c:1392` gates the `sw_filt_ref_adj_*`
   register writes on `mode_ref_delta_enabled`. If that flag disagrees on a
   lossless/intrabc frame (case 2), the captured registers will be all zero on
   the MPP side and populated with default deltas on the regbuilder side, or
   vice-versa.

**Cross-frame persistence note.** Neither parser carries
`loop_filter_delta_enabled` *itself* as sticky state. What is sticky is the
ref_delta/mode_delta *values* attached to each saved reference slot
(`AV1ReferenceFrameState::loop_filter_ref_deltas/_mode_deltas`, written at
`av1d_cbs.c:2132-2135` whenever `refresh_frame_flags` updates that slot).
dav1d achieves the same effect by holding a `Dav1dFrameHeader` ref per stored
slot.

## 3. Spec-level reconstruction (clean-room reading list)

For a clean-room parser, here is the minimal description of each disputed
field, expressed purely in AV1 spec terms.

### 3.1 `reference_select` (DXVA `coding.reference_mode`)

- Section 5.9.20 *frame_reference_mode()*.
- Read order: immediately after `tx_mode` and before `skip_mode_params()`.
- If `FrameIsIntra` is true: `reference_select = 0` (inferred, no bits read).
- Else: read `f(1)` into `reference_select`.
- DXVA: copy `reference_select` straight through to `coding.reference_mode`.

### 3.2 `skip_mode_present` (DXVA `coding.skip_mode`) and `skip_ref0/1`

- Section 5.9.21 *skip_mode_params()*.
- Read order: immediately after *frame_reference_mode()*.
- Compute `SkipModeAllowed` (§5.9.22) using order-hint distances against the
  current frame's seven *reference frames* (in `ref_frame_idx[0..6]` order),
  selecting nearest forward and nearest backward by signed order-hint distance;
  fall back to two-forward selection if no backward exists.
- If `SkipModeAllowed`: read `f(1)` into `skip_mode_present`; else infer 0.
- The two reference-frame indices that the spec exposes as
  `SkipModeFrame[ 0 ]` and `SkipModeFrame[ 1 ]` are the spec-correct values
  for `pp->skip_ref0` and `pp->skip_ref1`, indexed in the **per-frame**
  ref-slot order (0..6), with the **+1** offset that the HAL expects (so that
  0 means "default to LAST"). They must be **reset to 0 at the start of each
  frame's parse** so that frames where `SkipModeAllowed == 0` produce 0 in
  both DXVA fields. (This is the chief place a clean-room parser must
  *not* mimic MPP — see §4.)

### 3.3 `loop_filter.mode_ref_delta_enabled`

- Section 5.9.11 *loop_filter_params()*.
- Read order: immediately after `delta_lf_params` and before `cdef_params`.
- If `CodedLossless || allow_intrabc`: do not read; surface
  `mode_ref_delta_enabled = 1` and use the §5.9.11 default
  `loop_filter_ref_deltas[]` and `loop_filter_mode_deltas[]`.
  (This matches dav1d's behavior; MPP surfaces 0 here, which is the bug.)
- Else: read `f(1)` into `mode_ref_delta_enabled`. If it is 1, optionally
  read `loop_filter_delta_update` and per-ref / per-mode update bits + signed
  6-bit deltas. If 0, inherit from `primary_ref_frame`'s stored deltas (or
  defaults when `primary_ref_frame == PRIMARY_REF_NONE`).

## 4. Cross-frame persistence map

Per-frame fields in MPP that **survive across frames** without being reset at
frame start, sorted by relevance to the three disputed registers:

| Field                                       | Container                | Reset?                                   | Notes                                                                 |
|---------------------------------------------|--------------------------|------------------------------------------|-----------------------------------------------------------------------|
| `skip_ref0`, `skip_ref1`                    | `AV1Context`             | **No**                                   | Only written when `SkipModeAllowed`. **Confirmed sticky.** §2.1.       |
| `loop_filter_ref_deltas[]` (per-slot)       | `AV1ReferenceFrameState` | Updated when `refresh_frame_flags` hits  | Spec-correct sticky; both parsers persist it. §2.3.                    |
| `loop_filter_mode_deltas[]` (per-slot)      | `AV1ReferenceFrameState` | Same as above                            | Same.                                                                  |
| `feature_enabled[][]`, `feature_value[][]`  | `AV1ReferenceFrameState` | Same as above                            | Segmentation params, also spec-correct sticky.                         |
| `coded_lossless`, `all_lossless`            | `AV1Context`             | Recomputed each frame at lines 2054-2072 | Not sticky.                                                            |
| `disable_frame_end_update_cdf`              | `AV1Context`             | Recomputed at line 2010                  | Not sticky.                                                            |
| `frame_is_intra`                            | `AV1Context`             | Set at line 1811                         | Not sticky.                                                            |
| `refresh_frame_flags`                       | `AV1Context`             | Set at line 1929                         | Not sticky.                                                            |
| `ref_s[i].order_hint` etc.                  | `AV1ReferenceFrameState` | Updated when slot refreshed (line 2118)  | Spec-correct sticky.                                                   |
| `ref_s[i].valid`                            | `AV1ReferenceFrameState` | Cleared on KEY+show (line 1840) and on   | Spec-correct.                                                           |
|                                             |                          | id-based invalidation (line 1872, 1878)  |                                                                        |
| `current` (`AV1RawFrameHeader`)             | per-OBU `AV1RawOBU`      | **Yes** — `mpp_calloc` per OBU (2774)    | All per-frame syntax elements start at 0.                              |

**The user's three questions reduce to: `skip_ref0/1` is the only field in the
parser-output struct that MPP carries across frames in a way that dav1d does
not.** `reference_select` is fully per-frame in both parsers; `mode_ref_delta_enabled`
is fully per-frame in both parsers (only its associated *delta values* are
sticky, and that stickiness is spec-mandated).

## 5. Surprises and "don't copy these" notes

For the clean-room implementer:

1. **`skip_ref0/1` placement.** MPP keeps these in the persistent context, not
   in the per-frame header struct. If you mirror that placement *and* forget
   to zero them per frame, you will reproduce MPP's stale-value bug. The spec
   defines `SkipModeFrame[ 0 / 1 ]` as a per-frame derived quantity; they
   should reset to 0 at the start of every uncompressed_header parse and only
   be assigned inside the `SkipModeAllowed` branch.
2. **DXVA-fill order.** MPP fills `pp->skip_ref0/1` *unconditionally* from
   sticky context (av1d_parser2_syntax.c:270-271). A clean-room
   implementation should either gate the copy on `skip_mode_present` (or on
   `SkipModeAllowed`), or zero-and-overwrite per frame at the parser layer.
3. **`mode_ref_delta_enabled` in lossless / intrabc paths.** MPP surfaces 0
   here despite §5.9.11's default-value rule. dav1d surfaces 1. The
   spec-correct value is 1 (since the default ref/mode delta table is in use).
   A clean-room parser should follow dav1d here.
4. **HAL clamps mask the bug.** `(skip_ref{0,1} != 0) ? skip_ref{0,1} : 1`
   means a stale 5 looks like a deliberate 5; a properly zeroed
   `skip_ref{0,1}` looks like the LAST default (1). This makes the
   "regbuilder vs. capture" diff appear larger than the parser-bit divergence
   actually is.
5. **No order-of-operations weirdness.** MPP's *uncompressed_header()* honors
   the spec's syntax order. There are no reads of bits the spec does not list,
   and no fields are computed from non-bitstream state in a way that diverges
   from the spec. The only "from non-bitstream state" surfaces in DXVA are
   `seq->...` fields (sequence-header inheritance) and the persistent
   `skip_ref0/1` / per-slot loop-filter deltas described above.
6. **The original cited line ranges (`av1d_codec.c:1212-1295`,
   `hal_av1d_vdpu.c:2092-2093`) do not match this MPP copy.** They appear to
   be from a different MPP version. Behavior described in this document was
   verified against `av1d_cbs.c` (parser) and `av1d_parser2_syntax.c` (DXVA
   fill) in the local checkout. The HAL formula
   `sw_skip_ref{0,1} = (dxva->skip_ref{0,1} != 0) ? dxva->skip_ref{0,1} : 1`
   was not verified against local source (no `hal_av1d_vdpu.c` is present in
   this checkout) and is taken at face value from the user's evidence.

## 6. Bottom line for the regbuilder

| Symptom                                  | Root cause                                                                                                                       | Fix on the regbuilder side                                                                       |
|------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------|
| `sw_skip_ref1 = 0x14` while we emit 0x04 | MPP leaks stale `skip_ref0/1` from a prior frame; the HAL clamp turns "5" into a register write of 5. dav1d correctly emits 0.   | Match dav1d (per-frame zero). The captured BSP value is **wrong**, not a different parse. Do not chase it. |
| `sw_comp_pred_mode` divergence           | `reference_mode` in DXVA is the raw `reference_select` bit; both parsers read the same bit. Likely a regbuilder-side derivation mismatch. | Use `hdr->switchable_comp_refs` directly (1 bit per spec §5.9.20), apply the intra-frame infer-to-0, then use `(value != 0) ? 2 : 0`. |
| `sw_filt_ref_adj_*` divergence           | Either lossless/intrabc transient (MPP surfaces 0 vs. spec-correct 1), or stale per-slot deltas via mismatched primary-ref state. | Follow the spec's lossless/intrabc default. For inherited deltas, ensure the primary-ref slot's stored delta values are kept in sync with dav1d's `c->refs[i].p.p.frame_hdr` lifecycle. |

In all three cases the bitstream interpretation between MPP and dav1d agrees on
the bits. The disagreements are in **what gets surfaced to the consumer when a
syntax element is not present** (skip_ref staleness, lossless `mode_ref_delta_enabled`
default) and in **how the consumer derives a HAL register value from the parsed
field** (the comp_pred_mode case).

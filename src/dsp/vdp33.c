/*
 * VDP33 - vocoder / strings / human-voice ensemble synthesizer for Schwung
 *
 * SPDX-License-Identifier: MIT
 *
 * A from-scratch DSP model of the Behringer VC340 / Roland VP-330 instrument
 * architecture. The control layout follows the VC340 Quick Start Guide; the
 * sound itself is modelled from public descriptions. It contains no code,
 * samples or ROM data from any original instrument. README.md lists what the
 * manual confirms and which values are estimates.
 *
 * Panel -> parameter map (VC340 numbering)
 *
 *   Balance   : Direct Mic (6), Human Voice (7), Strings (8)   -- no vocoder fader
 *   Vocoder   : Tone (12), Mic Level (13), Lower (14), Upper (15), Ensemble (16)
 *   Human Voice: Male 8' Lower (17), Male 4' Lower (18), Male 8' Upper (19),
 *               Female 4' Upper (20), Ensemble (21), Attack (22)
 *   Release   : one fader for all three sections (23)
 *   Strings   : Attack (24), Tone (25), Lower (26), Upper (27)
 *   Vibrato   : Rate (9), Delay (10), Depth (11) -- Vocoder and Human Voice only
 *   Pitch shift: AUTO / MANUAL / OFF(pedal) (31), Down/Normal (32),
 *               Pitch Set (33), Time (34); Tune (29)
 *   Footswitches: Vocoder Hold -> CC 64, Pitch Shift -> CC 66
 *
 * Signal flow
 *
 *   keys -> two banks of 12 master oscillators (top octave, 32.32 fixed point)
 *           bank S (strings) : tune, bend
 *           bank V (voice/vocoder): tune, bend, vibrato, pitch shift
 *        -> octave dividers: phase-locked 16' / 8' / 4' registers
 *        -> per-key attack/release envelopes, one per voice type
 *   Strings : bus -> Tone low-pass -> Ensemble (always on)
 *   Voice   : Male 8'/4' bus -> male formants, Female 4' bus -> female formants
 *   Vocoder : carrier bus x 10-band analysis of mic/line-in -> Tone
 *
 * Realtime rules (docs/MODULES.md): no allocation, I/O or logging in any entry
 * point except create/destroy_instance.
 */

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "plugin_api_v1.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TWO_PI_F 6.28318530717958647692f

#define VDP_MAX_KEYS 32
#define VDP_BANDS    10
#define ENS_BUF      2048              /* power of two */
#define ENS_MASK     (ENS_BUF - 1)
#define INV_2_32     2.3283064365386963e-10f
#define PS_MAX_SEMIS 12.0f             /* Pitch Set fully at "L" (estimate) */

/* envelope channels: one per voice type */
enum { S_STR, S_VOC, S_M8, S_M4, S_F4, S_COUNT };
enum { ZN_OFF, ZN_LOWER, ZN_UPPER, ZN_ALL };
enum { PSM_PEDAL, PSM_AUTO, PSM_MANUAL };

/* ------------------------------------------------------------------ params */

enum {
    P_VOLUME, P_LVL_STR, P_LVL_VOI, P_DIRECT,
    P_ZN_VOC, P_ZN_STR, P_HV_M8L, P_HV_M4L, P_HV_M8U, P_HV_F4U, P_SPLIT,
    P_ATT_STR, P_ATT_VOI, P_REL, P_TONE, P_VOC_TONE, P_MIC,
    P_ENS_VOI, P_ENS_VOC, P_ENS_DEPTH, P_ENS_RATE,
    P_VIB_RATE, P_VIB_DEPTH, P_VIB_DELAY,
    P_PS_MODE, P_PS_SET, P_PS_TIME, P_PS_FADER,
    P_TUNE, P_VELOCITY, P_VOC_PEDAL, P_SIBIL, P_BEND, P_OCT,
    P_COUNT
};

typedef enum { T_FLOAT, T_INT, T_ENUM } ptype_t;

typedef struct {
    const char *key, *name, *sname;
    ptype_t type;
    float min, max, def;
    const char *const *opts;
    int nopts;
} pdef_t;

static const char *const OPT_ZONE[]  = { "Off", "Lower", "Upper", "All" };
static const char *const OPT_ONOFF[] = { "Off", "On" };
static const char *const OPT_PS[]    = { "Off/Pedal", "Auto", "Manual" };
static const char *const OPT_VEL[]  = { "Fixed", "Dynamic" };
static const char *const OPT_PEDAL[] = { "Off", "Gate" };

#define PF(k, n, s, d)         { k, n, s, T_FLOAT, 0.0f, 1.0f, d, NULL, 0 }
#define PI(k, n, s, mn, mx, d) { k, n, s, T_INT, (float)(mn), (float)(mx), (float)(d), NULL, 0 }
#define PE(k, n, s, o, cnt, d) { k, n, s, T_ENUM, 0.0f, (float)((cnt) - 1), (float)(d), o, cnt }

static const pdef_t g_params[P_COUNT] = {
    [P_VOLUME]    = PF("volume",           "Volume",              "Vol",   0.8f),
    [P_LVL_STR]   = PF("lvl_str",          "Strings Level",       "Strg",  0.0f),
    [P_LVL_VOI]   = PF("lvl_voice",        "Human Voice Level",   "Voice", 0.0f),
    [P_DIRECT]    = PF("direct_mic",       "Direct Mic Level",    "Dirmc", 0.0f),
    [P_ZN_VOC]    = PE("zone_voc",         "Vocoder Keys",        "ZnVoc", OPT_ZONE, 4, ZN_OFF),
    [P_ZN_STR]    = PE("zone_str",         "Strings Keys",        "ZnStr", OPT_ZONE, 4, ZN_OFF),
    [P_HV_M8L]    = PE("hv_m8_low",        "Voice Male 8' Lower", "M8Lo",  OPT_ONOFF, 2, 0),
    [P_HV_M4L]    = PE("hv_m4_low",        "Voice Male 4' Lower", "M4Lo",  OPT_ONOFF, 2, 0),
    [P_HV_M8U]    = PE("hv_m8_up",         "Voice Male 8' Upper", "M8Up",  OPT_ONOFF, 2, 0),
    [P_HV_F4U]    = PE("hv_f4_up",         "Voice Female 4' Upper","F4Up", OPT_ONOFF, 2, 0),
    [P_SPLIT]     = PI("split",            "Split Point",         "Split", 36, 84, 60),
    [P_ATT_STR]   = PF("att_str",          "Strings Attack",      "StrAt", 0.25f),
    [P_ATT_VOI]   = PF("att_voice",        "Human Voice Attack",  "VoiAt", 0.30f),
    [P_REL]       = PF("release",          "Release (all)",       "Rel",   0.30f),
    [P_TONE]      = PF("tone",             "Strings Tone",        "Tone",  0.50f),
    [P_VOC_TONE]  = PF("voc_tone",         "Vocoder Tone",        "VcTon", 0.80f),
    [P_MIC]       = PF("mic_gain",         "Mic Level",           "Mic",   0.50f),
    [P_ENS_VOI]   = PE("ens_voice",        "Voice Ensemble",      "EnVoi", OPT_ONOFF, 2, 1),
    [P_ENS_VOC]   = PE("ens_voc",          "Vocoder Ensemble",    "EnVoc", OPT_ONOFF, 2, 0),
    [P_ENS_DEPTH] = PF("ens_depth",        "Ensemble Depth",      "EnDpt", 0.70f),
    [P_ENS_RATE]  = PF("ens_rate",         "Ensemble Speed",      "EnRat", 0.50f),
    [P_VIB_RATE]  = PF("vib_rate",         "Vibrato Rate",        "VbRat", 0.50f),
    [P_VIB_DEPTH] = PF("vib_depth",        "Vibrato Depth",       "VbDpt", 0.0f),
    [P_VIB_DELAY] = PF("vib_delay",        "Vibrato Delay",       "VbDly", 0.30f),
    [P_PS_MODE]   = PE("ps_mode",          "Pitch Shift Mode",    "PsMod", OPT_PS, 3, PSM_PEDAL),
    [P_PS_SET]    = PF("ps_set",           "Pitch Set",           "PsSet", 0.0f),
    [P_PS_TIME]   = PF("ps_time",          "Pitch Shift Time",    "PsTim", 0.40f),
    [P_PS_FADER]  = PF("ps_fader",         "Pitch Down (manual)", "PsFdr", 0.0f),
    [P_TUNE]      = PF("tune",             "Tune",                "Tune",  0.50f),
    [P_VELOCITY]  = PE("velocity",         "Velocity",            "Vel",   OPT_VEL, 2, 0),
    [P_VOC_PEDAL] = PE("voc_pedal",        "Vocoder Hold Pedal",  "Pedal", OPT_PEDAL, 2, 0),
    [P_SIBIL]     = PF("sibilance",        "Sibilance",           "Sibil", 0.0f),
    [P_BEND]      = PI("bend_range",       "Bend Range",          "Bend",  0, 12, 2),
    [P_OCT]       = PI("octave_transpose", "Octave",              "Oct",  -3,  3, 0),
};

/* ----------------------------------------------------------------- presets */

typedef struct { int idx; float v; } pv_t;
typedef struct { const char *name; pv_t v[24]; } preset_t;

#define END { -1, 0.0f }
#define ON 1.0f

static const preset_t g_presets[] = {
    { "Strings",          { {P_LVL_STR, .85f}, {P_ZN_STR, ZN_ALL}, {P_ATT_STR, .30f}, {P_REL, .35f}, END } },
    { "Slow Strings",     { {P_LVL_STR, .85f}, {P_ZN_STR, ZN_ALL}, {P_ATT_STR, .70f}, {P_REL, .60f},
                            {P_TONE, .40f}, END } },
    { "Bright Strings",   { {P_LVL_STR, .80f}, {P_ZN_STR, ZN_ALL}, {P_ATT_STR, .20f}, {P_REL, .30f},
                            {P_TONE, .80f}, {P_ENS_DEPTH, .80f}, END } },
    { "Male Choir 8'",    { {P_LVL_VOI, .85f}, {P_HV_M8L, ON}, {P_HV_M8U, ON}, {P_ATT_VOI, .50f},
                            {P_REL, .50f}, END } },
    { "Choir 8' + 4'",    { {P_LVL_VOI, .62f}, {P_HV_M8L, ON}, {P_HV_M4L, ON}, {P_HV_M8U, ON}, {P_HV_F4U, ON},
                            {P_ATT_VOI, .45f}, {P_REL, .50f}, END } },
    { "Female 4' + Strings", { {P_LVL_VOI, .80f}, {P_LVL_STR, .55f}, {P_HV_F4U, ON}, {P_ZN_STR, ZN_LOWER},
                            {P_ATT_VOI, .40f}, {P_ATT_STR, .35f}, {P_REL, .45f}, END } },
    { "Voice + Strings",  { {P_LVL_STR, .60f}, {P_LVL_VOI, .70f}, {P_ZN_STR, ZN_ALL}, {P_HV_M8L, ON},
                            {P_HV_M8U, ON}, {P_ATT_STR, .40f}, {P_ATT_VOI, .45f}, {P_REL, .45f}, END } },
    { "Vocoder",          { {P_ZN_VOC, ZN_ALL}, {P_MIC, .60f}, {P_REL, .20f}, END } },
    { "Vocoder Ensemble", { {P_ZN_VOC, ZN_ALL}, {P_MIC, .60f}, {P_REL, .20f}, {P_ENS_VOC, ON}, END } },
    { "Vocoder + Strings",{ {P_ZN_VOC, ZN_ALL}, {P_MIC, .60f}, {P_LVL_STR, .35f}, {P_ZN_STR, ZN_ALL},
                            {P_ATT_STR, .30f}, {P_REL, .30f}, END } },
    { "Split Str / Voc",  { {P_LVL_STR, .80f}, {P_ZN_STR, ZN_LOWER}, {P_ZN_VOC, ZN_UPPER}, {P_MIC, .60f},
                            {P_ATT_STR, .35f}, {P_REL, .30f}, END } },
    { "Split Voice / Str",{ {P_LVL_STR, .60f}, {P_LVL_VOI, .65f}, {P_ZN_STR, ZN_UPPER}, {P_HV_M8L, ON},
                            {P_HV_M4L, ON}, {P_ATT_STR, .35f}, {P_ATT_VOI, .40f}, {P_REL, .40f}, END } },
    { "Voice Vibrato",    { {P_LVL_VOI, .85f}, {P_HV_M8L, ON}, {P_HV_M8U, ON}, {P_ATT_VOI, .35f}, {P_REL, .40f},
                            {P_VIB_DEPTH, .35f}, {P_VIB_RATE, .55f}, {P_VIB_DELAY, .50f}, END } },
    { "Pitch Dive",       { {P_LVL_VOI, .85f}, {P_HV_M8L, ON}, {P_HV_M8U, ON}, {P_ATT_VOI, .10f}, {P_REL, .35f},
                            {P_PS_MODE, PSM_AUTO}, {P_PS_SET, .30f}, {P_PS_TIME, .40f}, END } },
    { "Full Mix",         { {P_LVL_STR, .55f}, {P_LVL_VOI, .55f}, {P_ZN_STR, ZN_ALL}, {P_HV_M8L, ON},
                            {P_HV_M8U, ON}, {P_ZN_VOC, ZN_ALL}, {P_MIC, .60f}, {P_ATT_STR, .35f},
                            {P_ATT_VOI, .40f}, {P_REL, .35f}, {P_ENS_VOC, ON}, END } },
};
#define NUM_PRESETS ((int)(sizeof(g_presets) / sizeof(g_presets[0])))

/* ------------------------------------------------------------------ filters */

typedef struct { float ic1, ic2, g, k, a1, a2, a3; } svf_t;

/* Topology-preserving SVF (Zavalishin). Band-pass output is normalised to
 * unity gain at its centre frequency. */
static void svf_set(svf_t *s, float fc, float q, float sr)
{
    if (fc > 0.45f * sr) fc = 0.45f * sr;
    float g = tanf((float)M_PI * fc / sr);
    float k = 1.0f / q;
    s->g = g;
    s->k = k;
    s->a1 = 1.0f / (1.0f + g * (g + k));
    s->a2 = g * s->a1;
    s->a3 = g * s->a2;
}

static inline float svf_bp(svf_t *s, float v0)
{
    float v3 = v0 - s->ic2;
    float v1 = s->a1 * s->ic1 + s->a2 * v3;
    float v2 = s->ic2 + s->a2 * s->ic1 + s->a3 * v3;
    s->ic1 = 2.0f * v1 - s->ic1;
    s->ic2 = 2.0f * v2 - s->ic2;
    return v1 * s->k;
}

static inline float flush_f(float x) { return fabsf(x) < 1e-20f ? 0.0f : x; }
static inline void svf_flush(svf_t *s) { s->ic1 = flush_f(s->ic1); s->ic2 = flush_f(s->ic2); }

/* Approximate centres of the ten fixed vocoder bands (Hz). The manual only
 * gives the architecture; the exact values are estimates, spaced about half an
 * octave apart across the speech range. */
static const float k_band_hz[VDP_BANDS] = {
    160.0f, 240.0f, 350.0f, 510.0f, 740.0f, 1070.0f, 1550.0f, 2250.0f, 3250.0f, 4700.0f
};
#define BAND_Q 4.0f   /* per stage; each band is two cascaded stages */

/* Choir formants (typical /a/ values): Male 8'/4' and Female 4'. */
static const float k_form_male[3]   = {  730.0f, 1090.0f, 2440.0f };
static const float k_form_female[3] = {  850.0f, 1220.0f, 2810.0f };
static const float k_form_q[3]      = {    7.5f,    9.0f,   12.0f };

/* --------------------------------------------------------------- instance */

typedef struct {
    uint8_t used, gate, src, pc;
    int8_t k;                 /* octave-divider depth of the 8' register */
    uint32_t age;
    float vel;                /* 1.0 when Velocity = Fixed */
    float env[S_COUNT];
} vkey_t;

typedef struct {
    float sr, inv_sr;
    float p[P_COUNT];
    int cur_preset;

    vkey_t keys[VDP_MAX_KEYS];
    uint32_t age_ctr;

    /* two banks of master oscillators: 32.32 fixed-point cycle accumulators */
    uint64_t acc_s[12], acc_v[12];
    double base_inc[12];      /* increment per sample at pitch ratio 1.0 */
    float base_dt[12];        /* same, in cycles/sample */

    /* pitch */
    float bend_semis, bend_cur;
    float vib_ph, vib_env;
    float ps_cur;             /* pitch-shift offset in semitones (<= 0) */
    int pedal, ps_pedal;
    float pedal_gate;

    /* strings tone, vocoder tone */
    float tone_lp1, tone_lp2, vt_lp1, vt_lp2;

    /* human voice formants: male bus and female bus */
    svf_t form_m[3], form_f[3];

    /* vocoder */
    svf_t an[VDP_BANDS][2], sy[VDP_BANDS][2];
    float venv[VDP_BANDS];
    float dc_x1, dc_y1;
    uint32_t noise;
    float nlp;

    /* ensemble */
    float dl[ENS_BUF];
    int wp;
    float ph_slow, ph_fast;
    float tap_lp[4];
} vdp_t;

static const host_api_v1_t *g_host = NULL;

/* ----------------------------------------------------------------- helpers */

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static void set_param_value(vdp_t *v, int idx, float val)
{
    const pdef_t *d = &g_params[idx];
    val = clampf(val, d->min, d->max);
    if (d->type != T_FLOAT) val = floorf(val + 0.5f);
    v->p[idx] = val;
}

static void apply_preset(vdp_t *v, int idx)
{
    if (idx < 0 || idx >= NUM_PRESETS) return;
    for (int i = 0; i < P_COUNT; i++) {
        /* performance settings survive a preset change */
        if (i == P_OCT || i == P_BEND || i == P_VOLUME || i == P_TUNE || i == P_VELOCITY) continue;
        v->p[i] = g_params[i].def;
    }
    for (int i = 0; g_presets[idx].v[i].idx >= 0; i++)
        set_param_value(v, g_presets[idx].v[i].idx, g_presets[idx].v[i].v);
    v->cur_preset = idx;
}

static int json_num(const char *json, const char *key, float *out)
{
    char pat[64];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p) return -1;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    char *end;
    float f = strtof(p, &end);
    if (end == p) return -1;
    *out = f;
    return 0;
}

typedef struct { char *b; int cap, len, ovf; } jb_t;

static void jb_add(jb_t *j, const char *fmt, ...)
{
    if (j->ovf) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(j->b + j->len, (size_t)(j->cap - j->len), fmt, ap);
    va_end(ap);
    if (n < 0 || n >= j->cap - j->len) { j->ovf = 1; return; }
    j->len += n;
}

static int param_index(const char *key)
{
    for (int i = 0; i < P_COUNT; i++)
        if (strcmp(key, g_params[i].key) == 0) return i;
    return -1;
}

/* An enum may arrive as its index or as its option name. */
static int parse_enum(const pdef_t *d, const char *val, float *out)
{
    if ((val[0] >= '0' && val[0] <= '9') || val[0] == '-') { *out = (float)atoi(val); return 0; }
    for (int i = 0; i < d->nopts; i++)
        if (strcmp(val, d->opts[i]) == 0) { *out = (float)i; return 0; }
    return -1;
}

static int zone_allows(int zone, int note, int split)
{
    switch (zone) {
    case ZN_ALL:   return 1;
    case ZN_LOWER: return note < split;
    case ZN_UPPER: return note >= split;
    default:       return 0;
    }
}

/* -------------------------------------------------------------- note logic */

static void all_notes_off(vdp_t *v)
{
    for (int i = 0; i < VDP_MAX_KEYS; i++) v->keys[i].gate = 0;
}

static void note_on(vdp_t *v, int src, int vel)
{
    int any_gate = 0;
    for (int i = 0; i < VDP_MAX_KEYS; i++)
        if (v->keys[i].used && v->keys[i].gate) { any_gate = 1; break; }
    if (!any_gate) v->vib_env = 0.0f;       /* vibrato delay restarts */

    /* Pitch shift: AUTO (or OFF with the pedal down) drops the pitch on every
     * key press, and it then glides back up over TIME. */
    int psm = (int)v->p[P_PS_MODE];
    if (psm == PSM_AUTO || (psm == PSM_PEDAL && v->ps_pedal))
        v->ps_cur = -PS_MAX_SEMIS * v->p[P_PS_SET];

    int slot = -1;
    for (int i = 0; i < VDP_MAX_KEYS; i++)
        if (v->keys[i].used && v->keys[i].src == src) { slot = i; break; }
    if (slot < 0)
        for (int i = 0; i < VDP_MAX_KEYS; i++)
            if (!v->keys[i].used) { slot = i; break; }
    if (slot < 0) {                          /* steal: oldest released, else oldest */
        uint32_t best = 0xFFFFFFFFu;
        for (int pass = 0; pass < 2 && slot < 0; pass++)
            for (int i = 0; i < VDP_MAX_KEYS; i++)
                if ((pass == 1 || !v->keys[i].gate) && v->keys[i].age < best) { best = v->keys[i].age; slot = i; }
    }
    if (slot < 0) slot = 0;

    int nt = src + 12 * (int)v->p[P_OCT];
    while (nt > 119) nt -= 12;
    while (nt < 0) nt += 12;

    vkey_t *k = &v->keys[slot];
    if (!(k->used && k->src == src)) memset(k->env, 0, sizeof k->env);
    k->used = 1;
    k->gate = 1;
    k->src = (uint8_t)src;
    k->pc = (uint8_t)(nt % 12);
    k->k = (int8_t)((108 + k->pc - nt) / 12);
    k->age = ++v->age_ctr;
    k->vel = (v->p[P_VELOCITY] > 0.5f) ? powf((float)clampf((float)vel, 1.0f, 127.0f) / 127.0f, 0.8f) : 1.0f;
}

static void note_off(vdp_t *v, int src)
{
    for (int i = 0; i < VDP_MAX_KEYS; i++)
        if (v->keys[i].used && v->keys[i].gate && v->keys[i].src == src) v->keys[i].gate = 0;
}

/* -------------------------------------------------------------- oscillators */

static inline float pblep(float t, float dt)
{
    if (t < dt) { t /= dt; return t + t - t * t - 1.0f; }
    if (t > 1.0f - dt) { t = (t - 1.0f) / dt; return t * t + t + t + 1.0f; }
    return 0.0f;
}

static inline float osc_saw(float p, float dt)
{
    if (dt > 0.45f) return 0.0f;
    return 2.0f * p - 1.0f - pblep(p, dt);
}

static inline float osc_sq(float p, float dt)
{
    if (dt > 0.45f) return 0.0f;
    float v = p < 0.5f ? 1.0f : -1.0f;
    float p2 = p + 0.5f;
    if (p2 >= 1.0f) p2 -= 1.0f;
    return v + pblep(p, dt) - pblep(p2, dt);
}

/* ------------------------------------------------------------ block setup */

/* Position 0 = no delay (manual: "the sound will reach maximum level
 * immediately"); the top of the travel is an estimate. */
static float att_time(float x) { return x < 0.005f ? 0.0f : 0.003f * powf(800.0f, x); }   /* ~3 ms .. 2.4 s */
static float rel_time(float x) { return x < 0.005f ? 0.0f : 0.015f * powf(400.0f, x); }   /* ~15 ms .. 6 s  */

static float coef_for_time(float t, float sr)
{
    float tau = t * (1.0f / 3.0f);
    if (tau < 3e-4f) tau = 3e-4f;            /* "immediate" still avoids a click */
    return 1.0f - expf(-1.0f / (sr * tau));
}

/* ----------------------------------------------------------------- render */

static void vdp_render(vdp_t *v, int16_t *out, int frames)
{
    const float sr = v->sr, inv_sr = v->inv_sr;
    const float *P = v->p;

    const int16_t *mic = NULL;
    if (g_host && g_host->mapped_memory)
        mic = (const int16_t *)(g_host->mapped_memory + g_host->audio_in_offset);

    /* --- per-block coefficients --- */
    const float ca[S_COUNT] = {
        coef_for_time(att_time(P[P_ATT_STR]), sr),
        coef_for_time(0.004f, sr),                         /* vocoder: no attack control */
        coef_for_time(att_time(P[P_ATT_VOI]), sr),
        coef_for_time(att_time(P[P_ATT_VOI]), sr),
        coef_for_time(att_time(P[P_ATT_VOI]), sr),
    };
    const float cr = coef_for_time(rel_time(P[P_REL]), sr);

    const float tone_a = 1.0f - expf(-TWO_PI_F * (400.0f * powf(30.0f, P[P_TONE])) * inv_sr);
    const float vt_a   = 1.0f - expf(-TWO_PI_F * (800.0f * powf(15.0f, P[P_VOC_TONE])) * inv_sr);

    const float mic_g = powf(10.0f, (-12.0f + 36.0f * P[P_MIC]) * 0.05f);
    const float direct_g = P[P_DIRECT] * P[P_DIRECT] * 1.0f;
    const int gate_mode = P[P_VOC_PEDAL] > 0.5f;
    const float pg_target = (!gate_mode || v->pedal) ? 1.0f : 0.0f;
    const float env_a = 1.0f - expf(-inv_sr / 0.003f);
    const float env_r = 1.0f - expf(-inv_sr / 0.045f);

    const float g_str = P[P_LVL_STR] * P[P_LVL_STR] * 0.40f;
    const float g_voi = P[P_LVL_VOI] * P[P_LVL_VOI] * 0.90f;
    const float g_voc = 1.10f;
    const float master = P[P_VOLUME];

    const int ens_v = P[P_ENS_VOI] > 0.5f, ens_c = P[P_ENS_VOC] > 0.5f;
    const float ens_scale = 0.5f + P[P_ENS_RATE];
    const float slow_inc = 0.55f * ens_scale * inv_sr, fast_inc = 5.9f * ens_scale * inv_sr;
    const float slow_amp = 0.0021f * P[P_ENS_DEPTH], fast_amp = 0.00035f * P[P_ENS_DEPTH];
    const float ens_lp_a = 1.0f - expf(-TWO_PI_F * 5500.0f * inv_sr);

    const float vib_hz = 2.0f + 7.0f * P[P_VIB_RATE];
    const float vib_st = 0.6f * P[P_VIB_DEPTH];
    const float vib_inc = inv_sr / (0.02f + 2.0f * P[P_VIB_DELAY]);

    const int psm = (int)P[P_PS_MODE];
    const float ps_range = PS_MAX_SEMIS * P[P_PS_SET];
    const float ps_glide = 1.0f - expf(-inv_sr / ((0.03f * powf(60.0f, P[P_PS_TIME])) * (1.0f / 3.0f)));
    const float tune_semis = (P[P_TUNE] - 0.5f) * 2.0f;     /* +-1 semitone */

    /* --- active keys and their targets --- */
    int act[VDP_MAX_KEYS], nact = 0, any_gate = 0;
    float tgt[VDP_MAX_KEYS][S_COUNT];
    const int split = (int)P[P_SPLIT];
    for (int i = 0; i < VDP_MAX_KEYS; i++) {
        vkey_t *k = &v->keys[i];
        if (!k->used) continue;
        const int low = k->src < split;
        float g = k->gate ? k->vel : 0.0f;
        tgt[nact][S_STR] = zone_allows((int)P[P_ZN_STR], k->src, split) ? g : 0.0f;
        tgt[nact][S_VOC] = zone_allows((int)P[P_ZN_VOC], k->src, split) ? g : 0.0f;
        tgt[nact][S_M8]  = ((low && P[P_HV_M8L] > 0.5f) || (!low && P[P_HV_M8U] > 0.5f)) ? g : 0.0f;
        tgt[nact][S_M4]  = (low && P[P_HV_M4L] > 0.5f) ? g : 0.0f;
        tgt[nact][S_F4]  = (!low && P[P_HV_F4U] > 0.5f) ? g : 0.0f;
        if (k->gate) any_gate = 1;
        act[nact++] = i;
    }

    int voc_keys = 0;
    for (int a = 0; a < nact; a++)
        if (tgt[a][S_VOC] > 0.0f || v->keys[act[a]].env[S_VOC] > 1e-5f) { voc_keys = 1; break; }
    const int voc_on = (mic != NULL) && voc_keys && (pg_target > 0.0f || v->pedal_gate > 1e-3f);
    const int direct_on = (mic != NULL) && (direct_g > 1e-5f);

    static const float inv2k[12] = { 1.0f, 0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f,
                                     0.015625f, 0.0078125f, 0.00390625f, 0.001953125f,
                                     0.0009765625f, 0.00048828125f };

    for (int n = 0; n < frames; n++) {
        /* ---- pitch ---- */
        v->bend_cur += (v->bend_semis - v->bend_cur) * 0.01f;
        v->vib_ph += vib_hz * inv_sr;
        if (v->vib_ph >= 1.0f) v->vib_ph -= 1.0f;
        if (any_gate) { v->vib_env += vib_inc; if (v->vib_env > 1.0f) v->vib_env = 1.0f; }

        if (psm == PSM_MANUAL) v->ps_cur += (-ps_range * P[P_PS_FADER] - v->ps_cur) * 0.004f;
        else                   v->ps_cur += (0.0f - v->ps_cur) * ps_glide;

        const float semis_s = tune_semis + v->bend_cur;                      /* strings: no vibrato, no shift */
        const float semis_v = semis_s + v->ps_cur +
                              vib_st * v->vib_env * sinf(TWO_PI_F * v->vib_ph); /* voice + vocoder */
        const float mS = exp2f(semis_s * (1.0f / 12.0f));
        const float mV = exp2f(semis_v * (1.0f / 12.0f));
        for (int pc = 0; pc < 12; pc++) {
            v->acc_s[pc] += (uint64_t)(v->base_inc[pc] * (double)mS);
            v->acc_v[pc] += (uint64_t)(v->base_inc[pc] * (double)mV);
        }

        /* ---- keys: envelopes + divide-down oscillators ---- */
        float strB = 0.0f, vocB = 0.0f, m8B = 0.0f, m4B = 0.0f, f4B = 0.0f, vact = 0.0f;
        for (int a = 0; a < nact; a++) {
            vkey_t *k = &v->keys[act[a]];
            float emax = 0.0f, evoice = 0.0f;
            for (int s = 0; s < S_COUNT; s++) {
                float t = tgt[a][s], e = k->env[s];
                e += (t - e) * (t > e ? ca[s] : cr);
                k->env[s] = e;
                if (e > emax) emax = e;
                if (s >= S_VOC && e > evoice) evoice = e;
            }
            if (k->env[S_VOC] > vact) vact = k->env[S_VOC];
            if (emax < 1e-6f) continue;
            const int kk = k->k;

            if (k->env[S_STR] > 1e-6f) {
                const uint64_t A = v->acc_s[k->pc];
                const float dt8 = v->base_dt[k->pc] * mS * inv2k[kk];
                float s16 = osc_sq((float)(uint32_t)(A >> (kk + 1)) * INV_2_32, dt8 * 0.5f);
                float s8  = osc_saw((float)(uint32_t)(A >> kk) * INV_2_32, dt8);
                float s4  = (kk >= 1) ? osc_saw((float)(uint32_t)(A >> (kk - 1)) * INV_2_32, dt8 * 2.0f) : 0.0f;
                strB += (0.30f * s16 + 0.55f * s8 + 0.30f * s4) * k->env[S_STR];
            }
            if (evoice > 1e-6f) {
                const uint64_t A = v->acc_v[k->pc];
                const float dt8 = v->base_dt[k->pc] * mV * inv2k[kk];
                float v16 = osc_sq((float)(uint32_t)(A >> (kk + 1)) * INV_2_32, dt8 * 0.5f);
                float v8  = osc_saw((float)(uint32_t)(A >> kk) * INV_2_32, dt8);
                float v4  = (kk >= 1) ? osc_saw((float)(uint32_t)(A >> (kk - 1)) * INV_2_32, dt8 * 2.0f) : 0.0f;
                vocB += (0.35f * v16 + 0.60f * v8 + 0.40f * v4) * k->env[S_VOC];
                m8B  += (0.45f * v16 + 0.75f * v8) * k->env[S_M8];
                m4B  += (0.45f * v8  + 0.75f * v4) * k->env[S_M4];
                f4B  += (0.45f * v8  + 0.75f * v4) * k->env[S_F4];
            }
        }

        /* ---- Strings: Tone low-pass ---- */
        v->tone_lp1 += tone_a * (strB - v->tone_lp1);
        v->tone_lp2 += tone_a * (v->tone_lp1 - v->tone_lp2);
        float sStr = v->tone_lp2 * g_str;

        /* ---- Human Voice: Male 8'/4' through male formants, Female 4' through female ---- */
        float vm = m8B + m4B;
        float sVoi = (svf_bp(&v->form_m[0], vm) + 0.70f * svf_bp(&v->form_m[1], vm) +
                      0.40f * svf_bp(&v->form_m[2], vm) + 0.15f * vm +
                      svf_bp(&v->form_f[0], f4B) + 0.70f * svf_bp(&v->form_f[1], f4B) +
                      0.40f * svf_bp(&v->form_f[2], f4B) + 0.15f * f4B) * g_voi;

        /* ---- mic / line-in: modulator and direct signal ---- */
        float micx = 0.0f;
        if (voc_on || direct_on) {
            float x = ((float)mic[2 * n] + (float)mic[2 * n + 1]) * (0.5f / 32768.0f) * mic_g;
            micx = x - v->dc_x1 + 0.995f * v->dc_y1;               /* DC blocker */
            v->dc_x1 = x;
            v->dc_y1 = micx;
        }
        v->pedal_gate += (pg_target - v->pedal_gate) * 0.01f;

        /* ---- Vocoder: 10 fixed bands, modulator = mic / line-in ---- */
        float sVoc = 0.0f;
        if (voc_on) {
            float acc = 0.0f;
            for (int b = 0; b < VDP_BANDS; b++) {
                float bm = svf_bp(&v->an[b][1], svf_bp(&v->an[b][0], micx));
                float e = fabsf(bm);
                v->venv[b] += (e > v->venv[b] ? env_a : env_r) * (e - v->venv[b]);
                float cb = svf_bp(&v->sy[b][1], svf_bp(&v->sy[b][0], vocB));
                acc += cb * v->venv[b];
            }
            acc *= 3.2f;
            if (P[P_SIBIL] > 0.0f) {
                v->noise ^= v->noise << 13; v->noise ^= v->noise >> 17; v->noise ^= v->noise << 5;
                float nz = (float)(int32_t)v->noise * (1.0f / 2147483648.0f);
                v->nlp += 0.55f * (nz - v->nlp);
                float act_g = vact * 3.0f; if (act_g > 1.0f) act_g = 1.0f;
                acc += (nz - v->nlp) * (v->venv[8] + v->venv[9]) * P[P_SIBIL] * 2.5f * act_g;
            }
            /* Vocoder Tone */
            v->vt_lp1 += vt_a * (acc - v->vt_lp1);
            v->vt_lp2 += vt_a * (v->vt_lp1 - v->vt_lp2);
            sVoc = v->vt_lp2 * g_voc * v->pedal_gate;
        }
        float sDirect = direct_on ? micx * direct_g : 0.0f;

        /* ---- Ensemble: 2 delay lines per channel, slow + fast modulation ---- */
        float ens_in = sStr + (ens_v ? sVoi : 0.0f) + (ens_c ? sVoc : 0.0f);   /* strings: always */
        float dry    = (ens_v ? 0.0f : sVoi) + (ens_c ? 0.0f : sVoc) + sDirect;

        v->dl[v->wp] = ens_in;
        float wetL, wetR;
        v->ph_slow += slow_inc; if (v->ph_slow >= 1.0f) v->ph_slow -= 1.0f;
        v->ph_fast += fast_inc; if (v->ph_fast >= 1.0f) v->ph_fast -= 1.0f;
        {
            float ss = sinf(TWO_PI_F * v->ph_slow), sc = cosf(TWO_PI_F * v->ph_slow);
            float fs = sinf(TWO_PI_F * v->ph_fast), fc = cosf(TWO_PI_F * v->ph_fast);
            /* the four taps sit 90 degrees apart: sin, -sin, cos, -cos */
            const float ms[4] = { ss, -ss, sc, -sc }, mf[4] = { fs, -fs, fc, -fc };
            float tap[4];
            for (int j = 0; j < 4; j++) {
                float d = 0.0058f + slow_amp * ms[j] + fast_amp * mf[j];
                float rp = (float)v->wp - d * sr;
                if (rp < 0.0f) rp += (float)ENS_BUF;
                int i0 = (int)rp;
                float fr = rp - (float)i0;
                float x = v->dl[i0 & ENS_MASK] * (1.0f - fr) + v->dl[(i0 + 1) & ENS_MASK] * fr;
                v->tap_lp[j] += ens_lp_a * (x - v->tap_lp[j]);     /* BBD reconstruction filter */
                tap[j] = v->tap_lp[j];
            }
            wetL = 0.5f * (tap[0] + tap[1]);
            wetR = 0.5f * (tap[2] + tap[3]);
        }
        v->wp = (v->wp + 1) & ENS_MASK;

        /* dry/wet chosen so switching Ensemble on keeps the level within ~0.5 dB */
        float L = dry + 0.65f * ens_in + 1.10f * wetL;
        float R = dry + 0.65f * ens_in + 1.10f * wetR;
        L = tanhf(L * master);
        R = tanhf(R * master);
        out[2 * n]     = (int16_t)lrintf(L * 32767.0f);
        out[2 * n + 1] = (int16_t)lrintf(R * 32767.0f);
    }

    /* ---- block end: free silent keys, flush denormals ---- */
    for (int a = 0; a < nact; a++) {
        vkey_t *k = &v->keys[act[a]];
        int quiet = 1;
        for (int s = 0; s < S_COUNT; s++) if (k->env[s] >= 1e-4f) quiet = 0;
        if (!k->gate && quiet) k->used = 0;
    }
    v->tone_lp1 = flush_f(v->tone_lp1); v->tone_lp2 = flush_f(v->tone_lp2);
    v->vt_lp1 = flush_f(v->vt_lp1);     v->vt_lp2 = flush_f(v->vt_lp2);
    v->nlp = flush_f(v->nlp);
    v->dc_x1 = flush_f(v->dc_x1);       v->dc_y1 = flush_f(v->dc_y1);
    v->ps_cur = flush_f(v->ps_cur);
    v->pedal_gate = flush_f(v->pedal_gate);
    for (int i = 0; i < 4; i++) v->tap_lp[i] = flush_f(v->tap_lp[i]);
    for (int i = 0; i < 3; i++) { svf_flush(&v->form_m[i]); svf_flush(&v->form_f[i]); }
    for (int b = 0; b < VDP_BANDS; b++) {
        svf_flush(&v->an[b][0]); svf_flush(&v->an[b][1]);
        svf_flush(&v->sy[b][0]); svf_flush(&v->sy[b][1]);
        v->venv[b] = flush_f(v->venv[b]);
    }
}

/* -------------------------------------------------------------- plugin API */

static void vdp_init_state(vdp_t *v, float sr)
{
    v->sr = sr;
    v->inv_sr = 1.0f / sr;
    v->noise = 0x1234567u;
    v->pedal_gate = 1.0f;
    for (int pc = 0; pc < 12; pc++) {
        double f_top = 440.0 * pow(2.0, (double)(108 + pc - 69) / 12.0);
        v->base_inc[pc] = f_top / (double)sr * 4294967296.0;
        v->base_dt[pc] = (float)(f_top / (double)sr);
    }
    for (int b = 0; b < VDP_BANDS; b++)
        for (int s = 0; s < 2; s++) {
            svf_set(&v->an[b][s], k_band_hz[b], BAND_Q, sr);
            svf_set(&v->sy[b][s], k_band_hz[b], BAND_Q, sr);
        }
    for (int i = 0; i < 3; i++) {
        svf_set(&v->form_m[i], k_form_male[i], k_form_q[i], sr);
        svf_set(&v->form_f[i], k_form_female[i], k_form_q[i], sr);
    }
    for (int i = 0; i < P_COUNT; i++) v->p[i] = g_params[i].def;
    apply_preset(v, 0);
}

static void *v2_create_instance(const char *module_dir, const char *json_defaults)
{
    (void)module_dir; (void)json_defaults;
    vdp_t *v = (vdp_t *)calloc(1, sizeof(vdp_t));
    if (!v) return NULL;
    float sr = (g_host && g_host->sample_rate > 0) ? (float)g_host->sample_rate : 44100.0f;
    vdp_init_state(v, sr);
    return v;
}

static void v2_destroy_instance(void *instance) { free(instance); }

static void v2_on_midi(void *instance, const uint8_t *msg, int len, int source)
{
    vdp_t *v = (vdp_t *)instance;
    (void)source;
    if (!v || len < 3) return;
    switch (msg[0] & 0xF0) {
    case 0x90:
        if (msg[2] > 0) note_on(v, msg[1], msg[2]);
        else            note_off(v, msg[1]);      /* velocity 0 = note-off */
        break;
    case 0x80:
        note_off(v, msg[1]);
        break;
    case 0xB0:
        if (msg[1] == 64) v->pedal = msg[2] >= 64;            /* Vocoder Hold footswitch */
        else if (msg[1] == 66) v->ps_pedal = msg[2] >= 64;    /* Pitch Shift footswitch  */
        else if (msg[1] == 123 || msg[1] == 120) all_notes_off(v);
        break;
    case 0xE0: {
        int raw = ((msg[2] << 7) | msg[1]) - 8192;
        v->bend_semis = ((float)raw / 8192.0f) * v->p[P_BEND];
        break;
    }
    default: break;
    }
}

static void v2_set_param(void *instance, const char *key, const char *val)
{
    vdp_t *v = (vdp_t *)instance;
    if (!v || !key || !val) return;

    if (strcmp(key, "state") == 0) {
        float f;
        if (json_num(val, "preset", &f) == 0 && f >= 0 && f < NUM_PRESETS) v->cur_preset = (int)f;
        for (int i = 0; i < P_COUNT; i++)
            if (json_num(val, g_params[i].key, &f) == 0) set_param_value(v, i, f);
        return;
    }
    if (strcmp(key, "preset") == 0) {
        int idx = atoi(val);
        if (idx >= 0 && idx < NUM_PRESETS && idx != v->cur_preset) {
            all_notes_off(v);
            apply_preset(v, idx);
        }
        return;
    }
    if (strcmp(key, "all_notes_off") == 0) { all_notes_off(v); return; }

    int i = param_index(key);
    if (i < 0) return;
    float f;
    if (g_params[i].type == T_ENUM) {
        if (parse_enum(&g_params[i], val, &f) != 0) return;
    } else {
        f = (float)atof(val);
    }
    set_param_value(v, i, f);
}

static int get_chain_params(char *buf, int buf_len)
{
    jb_t j = { buf, buf_len, 0, 0 };
    jb_add(&j, "[{\"key\":\"preset\",\"name\":\"Preset\",\"type\":\"int\",\"min\":0,\"max\":%d}", NUM_PRESETS - 1);
    for (int i = 0; i < P_COUNT; i++) {
        const pdef_t *d = &g_params[i];
        jb_add(&j, ",{\"key\":\"%s\",\"name\":\"%s\",\"short_name\":\"%s\"", d->key, d->name, d->sname);
        if (d->type == T_ENUM) {
            jb_add(&j, ",\"type\":\"enum\",\"options\":[");
            for (int o = 0; o < d->nopts; o++) jb_add(&j, "%s\"%s\"", o ? "," : "", d->opts[o]);
            jb_add(&j, "],\"default\":%d}", (int)d->def);
        } else if (d->type == T_INT) {
            jb_add(&j, ",\"type\":\"int\",\"min\":%d,\"max\":%d,\"step\":1,\"default\":%d}",
                   (int)d->min, (int)d->max, (int)d->def);
        } else {
            jb_add(&j, ",\"type\":\"float\",\"min\":0,\"max\":1,\"step\":0.01,\"default\":%g}", d->def);
        }
    }
    jb_add(&j, "]");
    return j.ovf ? -1 : j.len;
}

static int get_ui_hierarchy(char *buf, int buf_len)
{
    static const char *h =
        "{\"modes\":null,\"levels\":{"
        "\"root\":{\"list_param\":\"preset\",\"count_param\":\"preset_count\",\"name_param\":\"preset_name\","
            "\"children\":null,"
            "\"knobs\":[\"volume\",\"lvl_str\",\"lvl_voice\",\"direct_mic\",\"mic_gain\",\"voc_tone\",\"tone\",\"release\"],"
            "\"params\":[{\"level\":\"balance\",\"label\":\"Balance / Tone\"},{\"level\":\"keys\",\"label\":\"Keyboard\"},"
            "{\"level\":\"envelope\",\"label\":\"Envelope\"},{\"level\":\"ensemble\",\"label\":\"Ensemble\"},"
            "{\"level\":\"vibrato\",\"label\":\"Vibrato\"},{\"level\":\"pitch\",\"label\":\"Pitch Shift\"}]},"
        "\"balance\":{\"children\":null,"
            "\"knobs\":[\"volume\",\"lvl_str\",\"lvl_voice\",\"direct_mic\",\"mic_gain\",\"voc_tone\",\"tone\",\"sibilance\"],"
            "\"params\":[\"volume\",\"lvl_str\",\"lvl_voice\",\"direct_mic\",\"mic_gain\",\"voc_tone\",\"tone\",\"sibilance\"]},"
        "\"keys\":{\"children\":null,"
            "\"knobs\":[\"zone_voc\",\"zone_str\",\"hv_m8_low\",\"hv_m4_low\",\"hv_m8_up\",\"hv_f4_up\",\"split\",\"velocity\"],"
            "\"params\":[\"zone_voc\",\"zone_str\",\"hv_m8_low\",\"hv_m4_low\",\"hv_m8_up\",\"hv_f4_up\",\"split\",\"velocity\"]},"
        "\"envelope\":{\"children\":null,"
            "\"knobs\":[\"att_str\",\"att_voice\",\"release\"],"
            "\"params\":[\"att_str\",\"att_voice\",\"release\"]},"
        "\"ensemble\":{\"children\":null,"
            "\"knobs\":[\"ens_voice\",\"ens_voc\",\"ens_depth\",\"ens_rate\"],"
            "\"params\":[\"ens_voice\",\"ens_voc\",\"ens_depth\",\"ens_rate\"]},"
        "\"vibrato\":{\"children\":null,"
            "\"knobs\":[\"vib_rate\",\"vib_delay\",\"vib_depth\"],"
            "\"params\":[\"vib_rate\",\"vib_delay\",\"vib_depth\"]},"
        "\"pitch\":{\"children\":null,"
            "\"knobs\":[\"ps_mode\",\"ps_set\",\"ps_time\",\"ps_fader\",\"tune\",\"bend_range\",\"octave_transpose\",\"voc_pedal\"],"
            "\"params\":[\"ps_mode\",\"ps_set\",\"ps_time\",\"ps_fader\",\"tune\",\"bend_range\",\"octave_transpose\",\"voc_pedal\"]}"
        "}}";
    int len = (int)strlen(h);
    if (len >= buf_len) return -1;
    memcpy(buf, h, (size_t)len + 1);
    return len;
}

static int v2_get_param(void *instance, const char *key, char *buf, int buf_len)
{
    vdp_t *v = (vdp_t *)instance;
    if (!v || !key || !buf || buf_len <= 0) return -1;

    if (strcmp(key, "preset") == 0)       return snprintf(buf, (size_t)buf_len, "%d", v->cur_preset);
    if (strcmp(key, "preset_count") == 0) return snprintf(buf, (size_t)buf_len, "%d", NUM_PRESETS);
    if (strcmp(key, "preset_name") == 0)  return snprintf(buf, (size_t)buf_len, "%s", g_presets[v->cur_preset].name);
    if (strcmp(key, "name") == 0)         return snprintf(buf, (size_t)buf_len, "VDP33");
    if (strcmp(key, "ui_hierarchy") == 0) return get_ui_hierarchy(buf, buf_len);
    if (strcmp(key, "chain_params") == 0) return get_chain_params(buf, buf_len);

    if (strcmp(key, "state") == 0) {
        jb_t j = { buf, buf_len, 0, 0 };
        jb_add(&j, "{\"preset\":%d", v->cur_preset);
        for (int i = 0; i < P_COUNT; i++) jb_add(&j, ",\"%s\":%.4f", g_params[i].key, v->p[i]);
        jb_add(&j, "}");
        return j.ovf ? -1 : j.len;
    }

    int i = param_index(key);
    if (i < 0) return -1;
    if (g_params[i].type == T_FLOAT) return snprintf(buf, (size_t)buf_len, "%.4f", v->p[i]);
    return snprintf(buf, (size_t)buf_len, "%d", (int)v->p[i]);
}

static int v2_get_error(void *instance, char *buf, int buf_len)
{
    (void)instance; (void)buf; (void)buf_len;
    return 0;
}

static void v2_render_block(void *instance, int16_t *out, int frames)
{
    vdp_t *v = (vdp_t *)instance;
    if (!v) { memset(out, 0, (size_t)frames * 2 * sizeof(int16_t)); return; }
    vdp_render(v, out, frames);
}

static plugin_api_v2_t g_api = {
    .api_version      = 2,
    .create_instance  = v2_create_instance,
    .destroy_instance = v2_destroy_instance,
    .on_midi          = v2_on_midi,
    .set_param        = v2_set_param,
    .get_param        = v2_get_param,
    .get_error        = v2_get_error,
    .render_block     = v2_render_block,
};

plugin_api_v2_t *move_plugin_init_v2(const host_api_v1_t *host)
{
    g_host = host;
    return &g_api;
}

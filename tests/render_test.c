/*
 * Offline test + demo renderer for VDP33.
 *
 *   bash scripts/test.sh [output_dir]
 *
 * Includes vdp33.c directly so the checks can reach its internals. A synthetic
 * "speech" signal (a glottal saw through moving formants) is written into the
 * fake host's audio-input buffer, exactly where the real host puts mic/line-in.
 */
#include <time.h>
#include "../src/dsp/vdp33.c"

#define SR 44100
#define BLK 128

static uint8_t g_mem[8192];
static host_api_v1_t g_h;
static plugin_api_v2_t *api;

static int g_fail = 0;
#define CHECK(cond, ...) do { int ok_ = (cond); printf("  [%s] ", ok_ ? "ok" : "FAIL"); \
    printf(__VA_ARGS__); printf("\n"); if (!ok_) g_fail++; } while (0)

/* ---------------------------------------------------------- speech source */

typedef struct { float f[3]; } vowel_t;
static const vowel_t VOW[3] = { {{730, 1090, 2440}}, {{270, 2290, 3010}}, {{300, 870, 2240}} };  /* a i u */

typedef struct { svf_t s[3]; float ph; int cur; } speech_t;

static void speech_vowel(speech_t *sp, int v)
{
    if (sp->cur == v) return;
    sp->cur = v;
    for (int i = 0; i < 3; i++) svf_set(&sp->s[i], VOW[v].f[i], 9.0f, SR);
}

/* vowel schedule: 0.5 s voiced + 0.1 s gap, cycling a, i, u */
static int vowel_at(double t, float *amp)
{
    double c = fmod(t, 0.6);
    *amp = (c < 0.5) ? 0.45f : 0.0f;
    return ((int)(t / 0.6)) % 3;
}

static void speech_block(speech_t *sp, double t0, int16_t *dst, float *mon)
{
    for (int i = 0; i < BLK; i++) {
        float amp;
        int v = vowel_at(t0 + (double)i / SR, &amp);
        speech_vowel(sp, v);
        sp->ph += 120.0f / SR; if (sp->ph >= 1.0f) sp->ph -= 1.0f;
        float src = 2.0f * sp->ph - 1.0f;
        float y = svf_bp(&sp->s[0], src) + 0.7f * svf_bp(&sp->s[1], src) + 0.4f * svf_bp(&sp->s[2], src);
        y *= amp;
        dst[2 * i] = dst[2 * i + 1] = (int16_t)(clampf(y, -1.0f, 1.0f) * 32767.0f);
        if (mon) mon[i] = y;
    }
}

/* --------------------------------------------------------------- analysis */

static double band_energy(const float *x, int n, float flo, float fhi)
{
    double e = 0.0;
    const int K = 14;
    for (int k = 0; k < K; k++) {
        double f = flo * pow(fhi / flo, (double)k / (K - 1));
        double w = 2.0 * M_PI * f / SR, c = 2.0 * cos(w), s1 = 0, s2 = 0;
        for (int i = 0; i < n; i++) {
            double win = 0.5 - 0.5 * cos(2.0 * M_PI * i / (n - 1));
            double s0 = x[i] * win + c * s1 - s2; s2 = s1; s1 = s0;
        }
        e += s1 * s1 + s2 * s2 - c * s1 * s2;
    }
    return e;
}

static double peak_freq(const float *x, int n, double flo, double fhi, double step)
{
    double best = 0, bf = 0;
    for (double f = flo; f <= fhi; f += step) {
        double w = 2.0 * M_PI * f / SR, c = 2.0 * cos(w), s1 = 0, s2 = 0;
        for (int i = 0; i < n; i++) {
            double win = 0.5 - 0.5 * cos(2.0 * M_PI * i / (n - 1));
            double s0 = x[i] * win + c * s1 - s2; s2 = s1; s1 = s0;
        }
        double p = s1 * s1 + s2 * s2 - c * s1 * s2;
        if (p > best) { best = p; bf = f; }
    }
    return bf;
}

static void stats(const float *x, int n, double *peak, double *rms_db)
{
    double pk = 0, ss = 0;
    for (int i = 0; i < n; i++) { double a = fabs(x[i]); if (a > pk) pk = a; ss += (double)x[i] * x[i]; }
    *peak = pk;
    *rms_db = 20.0 * log10(sqrt(ss / (n > 0 ? n : 1)) + 1e-12);
}

static double rms_of(const float *x, int n) { double pk, r; stats(x, n, &pk, &r); return r; }

/* ------------------------------------------------------------------- run */

typedef struct { float *L, *R; int n, cap; } rec_t;

static void rec_init(rec_t *r, double secs)
{
    r->cap = (int)(secs * SR) + BLK;
    r->L = (float *)calloc((size_t)r->cap, sizeof(float));
    r->R = (float *)calloc((size_t)r->cap, sizeof(float));
    r->n = 0;
}
static void rec_free(rec_t *r) { free(r->L); free(r->R); }

/* type: 1 note on (a = note, b = velocity), 0 note off, 2 CC64 (a = value), 3 CC66 (a = value) */
typedef struct { double t; int type; int a; int b; } ev_t;

static void send(void *inst, int status, int a, int b)
{
    uint8_t m[3] = { (uint8_t)status, (uint8_t)a, (uint8_t)b };
    api->on_midi(inst, m, 3, 0);
}

static void run(void *inst, double secs, const ev_t *ev, int nev, int speech, rec_t *r, float *mic_mon)
{
    speech_t sp; memset(&sp, 0, sizeof sp); sp.cur = -1;
    int blocks = (int)(secs * SR) / BLK, e = 0;
    int16_t *mic = (int16_t *)(g_mem + MOVE_AUDIO_IN_OFFSET);
    int16_t out[BLK * 2];
    for (int b = 0; b < blocks; b++) {
        double t0 = (double)b * BLK / SR;
        while (e < nev && ev[e].t <= t0 + 1e-9) {
            if (ev[e].type == 1) send(inst, 0x90, ev[e].a, ev[e].b ? ev[e].b : 100);
            else if (ev[e].type == 0) send(inst, 0x80, ev[e].a, 0);
            else if (ev[e].type == 2) send(inst, 0xB0, 64, ev[e].a);
            else if (ev[e].type == 3) send(inst, 0xB0, 66, ev[e].a);
            e++;
        }
        if (speech) speech_block(&sp, t0, mic, mic_mon ? mic_mon + (size_t)b * BLK : NULL);
        else memset(mic, 0, BLK * 4);
        api->render_block(inst, out, BLK);
        if (r && r->n + BLK <= r->cap)
            for (int i = 0; i < BLK; i++) {
                r->L[r->n + i] = out[2 * i] / 32768.0f;
                r->R[r->n + i] = out[2 * i + 1] / 32768.0f;
            }
        if (r) r->n += BLK;
    }
}

static void write_wav(const char *path, const rec_t *r)
{
    FILE *f = fopen(path, "wb");
    if (!f) return;
    uint32_t data = (uint32_t)r->n * 4, riff = 36 + data, sr = SR, br = SR * 4, fmt = 16;
    uint16_t pcm = 1, ch = 2, ba = 4, bits = 16;
    fwrite("RIFF", 1, 4, f); fwrite(&riff, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f);
    fwrite(&fmt, 4, 1, f); fwrite(&pcm, 2, 1, f); fwrite(&ch, 2, 1, f);
    fwrite(&sr, 4, 1, f); fwrite(&br, 4, 1, f); fwrite(&ba, 2, 1, f); fwrite(&bits, 2, 1, f);
    fwrite("data", 1, 4, f); fwrite(&data, 4, 1, f);
    for (int i = 0; i < r->n; i++) {
        int16_t s[2] = { (int16_t)lrintf(clampf(r->L[i], -1, 1) * 32767.0f),
                         (int16_t)lrintf(clampf(r->R[i], -1, 1) * 32767.0f) };
        fwrite(s, 2, 2, f);
    }
    fclose(f);
}

static int preset_by_name(const char *name)
{
    for (int i = 0; i < NUM_PRESETS; i++) if (strcmp(g_presets[i].name, name) == 0) return i;
    return 0;
}

static void *new_inst(int preset)
{
    void *i = api->create_instance(".", NULL);
    char b[16]; snprintf(b, sizeof b, "%d", preset);
    api->set_param(i, "preset", b);
    return i;
}
static void setp(void *i, const char *k, const char *v) { api->set_param(i, k, v); }

/* a clean "voice only" instance: all sections off, then the caller turns on what it tests */
static void *blank_inst(void)
{
    void *i = new_inst(0);
    setp(i, "lvl_str", "0"); setp(i, "zone_str", "0"); setp(i, "lvl_voice", "0");
    setp(i, "ens_depth", "0");
    return i;
}

static const int CH[4] = { 48, 52, 55, 60 };   /* C3 E3 G3 C4 */
static void chord(ev_t *ev, int *n, double t_on, double t_off)
{
    *n = 0;
    for (int i = 0; i < 4; i++) ev[(*n)++] = (ev_t){ t_on + 0.01 * i, 1, CH[i], 100 };
    for (int i = 0; i < 4; i++) ev[(*n)++] = (ev_t){ t_off, 0, CH[i], 0 };
}

static int has_bad(const rec_t *r)
{
    for (int i = 0; i < r->n; i++) if (!isfinite(r->L[i]) || !isfinite(r->R[i])) return 1;
    return 0;
}

/* RMS (dBFS) of one held note, measured between 1.0 s and 1.5 s */
static double note_rms(void *in, int note, int vel, double *centroid_hz)
{
    rec_t r; rec_init(&r, 2.0);
    ev_t e[2] = { { 0.0, 1, note, vel }, { 1.8, 0, note, 0 } };
    run(in, 2.0, e, 2, 0, &r, NULL);
    double rms = rms_of(r.L + SR, SR / 2);
    if (centroid_hz) {
        double num = 0, den = 0;
        for (double f = 100; f < 6000; f += 25) {
            double en = band_energy(r.L + SR, SR / 2, (float)f, (float)(f * 1.0001));
            num += f * en; den += en;
        }
        *centroid_hz = den > 0 ? num / den : 0;
    }
    rec_free(&r);
    return rms;
}

int main(int argc, char **argv)
{
    const char *outdir = argc > 1 ? argv[1] : ".";
    memset(&g_h, 0, sizeof g_h);
    g_h.api_version = 1; g_h.sample_rate = SR; g_h.frames_per_block = BLK;
    g_h.mapped_memory = g_mem; g_h.audio_out_offset = MOVE_AUDIO_OUT_OFFSET; g_h.audio_in_offset = MOVE_AUDIO_IN_OFFSET;
    api = move_plugin_init_v2(&g_h);

    printf("VDP33 offline test (%d presets, %d params)\n", NUM_PRESETS, P_COUNT);
    char path[512];
    ev_t ev[16]; int nev;

    /* ---------------------------------------------------------------- 1 */
    printf("\n1. Intonazione (strings, ensemble depth 0)\n");
    {
        static const int notes[] = { 45, 57, 69, 81, 93 };
        for (int i = 0; i < 5; i++) {
            void *in = new_inst(0);
            setp(in, "ens_depth", "0");
            rec_t r; rec_init(&r, 1.6);
            ev_t e1[2] = { { 0.0, 1, notes[i], 100 }, { 1.5, 0, notes[i], 0 } };
            run(in, 1.6, e1, 2, 0, &r, NULL);
            double f_exp = 440.0 * pow(2.0, (notes[i] - 69) / 12.0);
            double f_got = peak_freq(r.L + SR / 2, SR, f_exp * 0.97, f_exp * 1.03, f_exp * 0.0004);
            CHECK(fabs(f_got - f_exp) < f_exp * 0.004, "nota %3d: atteso %.2f Hz, misurato %.2f Hz", notes[i], f_exp, f_got);
            rec_free(&r); api->destroy_instance(in);
        }
    }

    /* ---------------------------------------------------------------- 2 */
    printf("\n2. Livelli e stabilità su tutti i preset (accordo C3-E3-G3-C4, mic = voce sintetica)\n");
    for (int p = 0; p < NUM_PRESETS; p++) {
        void *in = new_inst(p);
        rec_t r; rec_init(&r, 6.0);
        chord(ev, &nev, 0.2, 4.0);
        run(in, 6.0, ev, nev, 1, &r, NULL);
        double pk, rms; stats(r.L + SR, 3 * SR, &pk, &rms);
        char nm[64]; api->get_param(in, "preset_name", nm, sizeof nm);
        printf("      %-20s picco %.2f  RMS %6.1f dBFS\n", nm, pk, rms);
        CHECK(!has_bad(&r) && pk < 0.98 && rms > -40.0, "preset %2d finito, non saturo, udibile", p);
        snprintf(path, sizeof path, "%s/demo_%02d_%s.wav", outdir, p, nm);
        for (char *c = path + strlen(outdir) + 1; *c; c++) if (*c == ' ' || *c == '/' || *c == '\'' || *c == '+') *c = '_';
        write_wav(path, &r);
        rec_free(&r); api->destroy_instance(in);
    }

    /* ---------------------------------------------------------------- 3 */
    printf("\n3. Vocoder: lo spettro dell'uscita segue le vocali del modulatore\n");
    {
        void *in = new_inst(preset_by_name("Vocoder"));
        rec_t r; rec_init(&r, 7.0);
        float *mon = (float *)calloc((size_t)r.cap, sizeof(float));
        chord(ev, &nev, 0.1, 6.5);
        run(in, 7.0, ev, nev, 1, &r, mon);
        const char *nmv[3] = { "a", "i", "u" };
        double ratio_out[3] = { 0 }, ratio_in[3] = { 0 };
        int cnt[3] = { 0 };
        for (int seg = 3; seg < 10; seg++) {
            double t0 = seg * 0.6 + 0.15;
            int n = (int)(0.3 * SR), i0 = (int)(t0 * SR), v = seg % 3;
            double hi_o = band_energy(r.L + i0, n, 2000, 3600), lo_o = band_energy(r.L + i0, n, 500, 1300);
            double hi_i = band_energy(mon + i0, n, 2000, 3600), lo_i = band_energy(mon + i0, n, 500, 1300);
            ratio_out[v] += 10 * log10((hi_o + 1e-30) / (lo_o + 1e-30));
            ratio_in[v]  += 10 * log10((hi_i + 1e-30) / (lo_i + 1e-30));
            cnt[v]++;
        }
        for (int v = 0; v < 3; v++) {
            ratio_out[v] /= cnt[v]; ratio_in[v] /= cnt[v];
            printf("      vocale /%s/: alti/bassi  ingresso %+6.1f dB   uscita %+6.1f dB\n", nmv[v], ratio_in[v], ratio_out[v]);
        }
        CHECK(ratio_out[1] > ratio_out[0] + 3.0 && ratio_out[1] > ratio_out[2] + 3.0,
              "/i/ (F2 alto) più brillante di /a/ e /u/ di >3 dB in uscita");
        void *in2 = new_inst(preset_by_name("Vocoder")); rec_t r2; rec_init(&r2, 2.0);
        chord(ev, &nev, 0.1, 1.9);
        run(in2, 2.0, ev, nev, 0, &r2, NULL);
        double rms2 = rms_of(r2.L + SR / 2, SR);
        CHECK(rms2 < -70.0, "mic muto => vocoder muto (RMS %.1f dBFS)", rms2);
        rec_free(&r2); api->destroy_instance(in2);
        free(mon); rec_free(&r); api->destroy_instance(in);
    }

    /* ---------------------------------------------------------------- 4 */
    printf("\n4. Pedale Vocoder Hold (CC 64) in modo Gate: il vocoder si sente solo a pedale premuto\n");
    {
        void *in = new_inst(preset_by_name("Vocoder"));
        setp(in, "voc_pedal", "Gate");
        rec_t r; rec_init(&r, 6.0);
        ev_t e[8]; int n = 0;
        for (int i = 0; i < 4; i++) e[n++] = (ev_t){ 0.0, 1, CH[i], 100 };
        e[n++] = (ev_t){ 2.0, 2, 127, 0 };           /* pedal down 2.0 - 4.0 s */
        e[n++] = (ev_t){ 4.0, 2, 0, 0 };
        run(in, 6.0, e, n, 1, &r, NULL);
        double a = rms_of(r.L + (int)(0.8 * SR), (int)(1.0 * SR));
        double b = rms_of(r.L + (int)(2.8 * SR), (int)(1.0 * SR));
        double c = rms_of(r.L + (int)(4.8 * SR), (int)(1.0 * SR));
        printf("      RMS prima %.1f dBFS, a pedale premuto %.1f dBFS, dopo il rilascio %.1f dBFS\n", a, b, c);
        CHECK(a < -70.0 && b > -40.0 && c < -70.0, "muto, udibile, di nuovo muto");
        rec_free(&r); api->destroy_instance(in);
    }

    /* ---------------------------------------------------------------- 5 */
    printf("\n5. Direct Mic: il microfono passa diretto, indipendente dal vocoder\n");
    {
        double lv[2];
        for (int d = 0; d < 2; d++) {
            void *in = blank_inst();
            setp(in, "direct_mic", d ? "0.8" : "0"); setp(in, "mic_gain", "0.5");
            rec_t r; rec_init(&r, 2.0);
            run(in, 2.0, NULL, 0, 1, &r, NULL);              /* speech, no keys */
            lv[d] = rms_of(r.L + SR / 2, SR);
            rec_free(&r); api->destroy_instance(in);
        }
        CHECK(lv[0] < -70.0 && lv[1] > -35.0, "direct mic 0 = muto (%.1f dBFS), 0.8 = voce passa (%.1f dBFS)", lv[0], lv[1]);
    }

    /* ---------------------------------------------------------------- 6 */
    printf("\n6. Zone Strings/Vocoder e quattro interruttori Human Voice (split a Do centrale)\n");
    {
        const int low = 48, high = 72;
        struct { const char *what; const char *k, *v; int note; int expect_sound; } T[] = {
            { "Strings Lower, nota bassa",  "zone_str", "1", low,  1 },
            { "Strings Lower, nota alta ",  "zone_str", "1", high, 0 },
            { "Strings Upper, nota alta ",  "zone_str", "2", high, 1 },
            { "Strings Upper, nota bassa",  "zone_str", "2", low,  0 },
            { "Strings Off              ",  "zone_str", "0", low,  0 },
        };
        for (size_t i = 0; i < sizeof T / sizeof T[0]; i++) {
            void *in = blank_inst();
            setp(in, "lvl_str", "0.8"); setp(in, T[i].k, T[i].v);
            double rms = note_rms(in, T[i].note, 100, NULL);
            CHECK((rms > -60.0) == T[i].expect_sound, "%s -> %s (%.1f dBFS)", T[i].what, T[i].expect_sound ? "suona" : "muto", rms);
            api->destroy_instance(in);
        }
        struct { const char *what; const char *sw; int note; int expect_sound; } H[] = {
            { "Male 8' Lower, nota bassa ", "hv_m8_low", low,  1 },
            { "Male 8' Lower, nota alta  ", "hv_m8_low", high, 0 },
            { "Male 4' Lower, nota bassa ", "hv_m4_low", low,  1 },
            { "Male 4' Lower, nota alta  ", "hv_m4_low", high, 0 },
            { "Male 8' Upper, nota alta  ", "hv_m8_up",  high, 1 },
            { "Male 8' Upper, nota bassa ", "hv_m8_up",  low,  0 },
            { "Female 4' Upper, nota alta", "hv_f4_up",  high, 1 },
            { "Female 4' Upper, nota bassa","hv_f4_up",  low,  0 },
        };
        for (size_t i = 0; i < sizeof H / sizeof H[0]; i++) {
            void *in = blank_inst();
            setp(in, "lvl_voice", "0.8"); setp(in, H[i].sw, "1");
            double rms = note_rms(in, H[i].note, 100, NULL);
            CHECK((rms > -60.0) == H[i].expect_sound, "%s -> %s (%.1f dBFS)", H[i].what, H[i].expect_sound ? "suona" : "muto", rms);
            api->destroy_instance(in);
        }
        /* 4' sits an octave above 8': compare energy at f0 and at 2*f0 for the same key */
        double e1[2], e2[2];
        for (int r4 = 0; r4 < 2; r4++) {
            void *in = blank_inst(); setp(in, "lvl_voice", "0.8"); setp(in, r4 ? "hv_m4_low" : "hv_m8_low", "1");
            rec_t q; rec_init(&q, 2.0);
            ev_t ee[2] = { { 0.0, 1, 48, 100 }, { 1.8, 0, 48, 0 } };
            run(in, 2.0, ee, 2, 0, &q, NULL);
            e1[r4] = band_energy(q.L + SR, SR / 2, 129.0f, 132.6f);     /* 130.8 Hz = C3 */
            e2[r4] = band_energy(q.L + SR, SR / 2, 259.0f, 264.0f);     /* 261.6 Hz = C4 */
            rec_free(&q); api->destroy_instance(in);
        }
        double d8 = 10 * log10(e2[0] / e1[0]), d4 = 10 * log10(e2[1] / e1[1]);
        CHECK(d4 > d8 + 12.0, "Male 4' un'ottava sopra l'8': rapporto 261/131 Hz %+.1f dB (8') contro %+.1f dB (4')", d8, d4);
        /* male and female formants differ: same pitch (130.8 Hz, dense harmonics), every key 'upper' */
        double ratio[2];
        for (int g = 0; g < 2; g++) {
            void *in = blank_inst(); setp(in, "lvl_voice", "0.8"); setp(in, "split", "36");
            setp(in, g ? "hv_f4_up" : "hv_m8_up", "1");
            int nt = g ? 36 : 48;                                 /* female 4' on C2 = male 8' on C3 */
            rec_t r; rec_init(&r, 2.0);
            ev_t e2[2] = { { 0.0, 1, nt, 100 }, { 1.8, 0, nt, 0 } };
            run(in, 2.0, e2, 2, 0, &r, NULL);
            ratio[g] = 10 * log10(band_energy(r.L + SR, SR / 2, 800, 1000) / band_energy(r.L + SR, SR / 2, 600, 800));
            rec_free(&r); api->destroy_instance(in);
        }
        CHECK(ratio[1] > ratio[0] + 2.0, "formante femminile piu' alto: rapporto 800-1000/600-800 Hz %+.1f dB contro %+.1f dB (maschile)", ratio[1], ratio[0]);
    }

    /* ---------------------------------------------------------------- 7 */
    printf("\n7. Vibrato agisce su Voice e Vocoder, non su Strings\n");
    {
        double spread[2];
        for (int sec = 0; sec < 2; sec++) {
            void *in = blank_inst();
            if (sec == 0) { setp(in, "lvl_str", "0.8"); setp(in, "zone_str", "3"); }
            else          { setp(in, "lvl_voice", "0.8"); setp(in, "hv_m8_up", "1"); }
            setp(in, "vib_depth", "1"); setp(in, "vib_delay", "0");
            rec_t q; rec_init(&q, 4.0);
            ev_t e3[2] = { { 0.0, 1, 69, 100 }, { 3.9, 0, 69, 0 } };
            run(in, 4.0, e3, 2, 0, &q, NULL);
            double fmin = 1e9, fmax = 0;
            for (int w = 1; w < 28; w++) {
                double f = peak_freq(q.L + w * (SR / 8), SR / 8, 425, 455, 0.2);
                if (f < fmin) fmin = f;
                if (f > fmax) fmax = f;
            }
            spread[sec] = fmax - fmin;
            rec_free(&q); api->destroy_instance(in);
        }
        printf("      escursione di frequenza su La4: Strings %.1f Hz, Human Voice %.1f Hz\n", spread[0], spread[1]);
        CHECK(spread[0] < 2.0, "Strings stabile con vibrato al massimo");
        CHECK(spread[1] > 4.0 && spread[1] < 40.0, "Human Voice vibra, udibile ma contenuto");
    }

    /* ---------------------------------------------------------------- 8 */
    printf("\n8. Pitch Shift (Auto / Manual / Off con pedale) e Tune\n");
    {
        /* AUTO: the note starts below pitch and glides up to normal */
        void *in = blank_inst();
        setp(in, "lvl_voice", "0.8"); setp(in, "hv_m8_up", "1");
        setp(in, "ps_mode", "Auto"); setp(in, "ps_set", "0.5"); setp(in, "ps_time", "0.8");
        rec_t r; rec_init(&r, 4.0);
        ev_t e[2] = { { 0.0, 1, 69, 100 }, { 3.9, 0, 69, 0 } };
        run(in, 4.0, e, 2, 0, &r, NULL);
        double f_early = peak_freq(r.L + (int)(0.05 * SR), SR / 10, 250, 470, 1.0);
        double f_late  = peak_freq(r.L + (int)(3.0 * SR), SR / 4, 425, 455, 0.2);
        CHECK(f_early < 440.0 * 0.86 && fabs(f_late - 440.0) < 2.0, "Auto: parte a %.0f Hz e arriva a %.1f Hz (Pitch Set 0.5 = -6 semitoni)", f_early, f_late);
        rec_free(&r); api->destroy_instance(in);

        /* MANUAL: fader sets a static shift, range set by Pitch Set */
        in = blank_inst();
        setp(in, "lvl_voice", "0.8"); setp(in, "hv_m8_up", "1");
        setp(in, "ps_mode", "Manual"); setp(in, "ps_set", "0.5"); setp(in, "ps_fader", "1");
        rec_init(&r, 3.0);
        ev_t e2[2] = { { 0.0, 1, 69, 100 }, { 2.9, 0, 69, 0 } };
        run(in, 3.0, e2, 2, 0, &r, NULL);
        double f_man = peak_freq(r.L + SR, SR, 300, 330, 0.2), f_exp = 440.0 * pow(2.0, -6.0 / 12.0);
        CHECK(fabs(f_man - f_exp) < 2.0, "Manual: %.1f Hz (atteso %.1f Hz)", f_man, f_exp);
        rec_free(&r); api->destroy_instance(in);

        /* OFF + pedal CC 66 */
        double f_nopedal = 0, f_pedal = 0;
        for (int p = 0; p < 2; p++) {
            in = blank_inst();
            setp(in, "lvl_voice", "0.8"); setp(in, "hv_m8_up", "1");
            setp(in, "ps_mode", "Off/Pedal"); setp(in, "ps_set", "0.5"); setp(in, "ps_time", "0.7");
            rec_init(&r, 2.0);
            ev_t e3[3]; int n3 = 0;
            if (p) e3[n3++] = (ev_t){ 0.0, 3, 127, 0 };
            e3[n3++] = (ev_t){ 0.1, 1, 69, 100 };
            e3[n3++] = (ev_t){ 1.9, 0, 69, 0 };
            run(in, 2.0, e3, n3, 0, &r, NULL);
            double f = peak_freq(r.L + (int)(0.15 * SR), SR / 10, 250, 470, 1.0);
            if (p) f_pedal = f; else f_nopedal = f;
            rec_free(&r); api->destroy_instance(in);
        }
        CHECK(f_nopedal > 430.0 && f_pedal < 400.0, "Off/Pedal: senza pedale %.0f Hz, con pedale %.0f Hz", f_nopedal, f_pedal);

        /* strings ignore pitch shift; Tune moves everything */
        in = blank_inst();
        setp(in, "lvl_str", "0.8"); setp(in, "zone_str", "3");
        setp(in, "ps_mode", "Manual"); setp(in, "ps_set", "1"); setp(in, "ps_fader", "1");
        rec_init(&r, 2.0);
        ev_t e4[2] = { { 0.0, 1, 69, 100 }, { 1.9, 0, 69, 0 } };
        run(in, 2.0, e4, 2, 0, &r, NULL);
        double f_str = peak_freq(r.L + SR / 2, SR, 425, 455, 0.2);
        CHECK(fabs(f_str - 440.0) < 1.5, "Strings non seguono il pitch shift (%.1f Hz con shift al massimo)", f_str);
        rec_free(&r); api->destroy_instance(in);

        in = blank_inst();
        setp(in, "lvl_str", "0.8"); setp(in, "zone_str", "3"); setp(in, "tune", "1");
        rec_init(&r, 2.0);
        run(in, 2.0, e4, 2, 0, &r, NULL);
        double f_tune = peak_freq(r.L + SR / 2, SR, 430, 470, 0.1);
        CHECK(fabs(f_tune - 440.0 * pow(2.0, 1.0 / 12.0)) < 1.0, "Tune al massimo: +1 semitono (%.1f Hz)", f_tune);
        rec_free(&r); api->destroy_instance(in);

        /* pitch bend */
        in = blank_inst();
        setp(in, "lvl_str", "0.8"); setp(in, "zone_str", "3");
        rec_init(&r, 1.0);
        ev_t e5[1] = { { 0.0, 1, 69, 100 } };
        run(in, 1.0, e5, 1, 0, &r, NULL);
        send(in, 0xE0, 0x7F, 0x7F);
        rec_t r2; rec_init(&r2, 1.0); run(in, 1.0, NULL, 0, 0, &r2, NULL);
        double f_b = peak_freq(r2.L + SR / 2, SR / 2, 440, 520, 0.05);
        double exp_b = 440.0 * pow(2.0, 2.0 * 8191.0 / 8192.0 / 12.0);
        CHECK(fabs(f_b - exp_b) < 1.0, "bend +2 st: %.2f Hz (atteso %.2f)", f_b, exp_b);
        rec_free(&r); rec_free(&r2); api->destroy_instance(in);
    }

    /* ---------------------------------------------------------------- 9 */
    printf("\n9. Velocity: Fixed ignora il tocco, Dynamic lo segue\n");
    {
        double lv[2][2];
        for (int d = 0; d < 2; d++)
            for (int k = 0; k < 2; k++) {
                void *in = blank_inst();
                setp(in, "lvl_str", "0.8"); setp(in, "zone_str", "3");
                setp(in, "velocity", d ? "Dynamic" : "Fixed");
                lv[d][k] = note_rms(in, 60, k ? 120 : 30, NULL);
                api->destroy_instance(in);
            }
        printf("      Fixed: %.1f / %.1f dBFS   Dynamic: %.1f / %.1f dBFS (vel 30 / 120)\n", lv[0][0], lv[0][1], lv[1][0], lv[1][1]);
        CHECK(fabs(lv[0][0] - lv[0][1]) < 0.5, "Fixed: stesso livello a vel 30 e 120");
        CHECK(lv[1][1] - lv[1][0] > 6.0, "Dynamic: >6 dB di differenza tra vel 30 e 120");
    }

    /* --------------------------------------------------------------- 10 */
    printf("\n10. Ensemble (Voice e Vocoder), attacco e rilascio a zero\n");
    {
        double lvl[2], var[2];
        for (int en = 0; en < 2; en++) {
            void *in = blank_inst();
            setp(in, "lvl_voice", "0.8"); setp(in, "hv_m8_low", "1"); setp(in, "ens_depth", "0.7");
            setp(in, "ens_voice", en ? "On" : "Off");
            rec_t q; rec_init(&q, 5.0);
            ev_t e2[2] = { { 0.0, 1, 48, 100 }, { 4.9, 0, 48, 0 } };
            run(in, 5.0, e2, 2, 0, &q, NULL);
            double d = 0, s = 0;
            for (int i = SR; i < 4 * SR; i++) { double df = q.L[i] - q.R[i]; d += df * df; s += (double)q.L[i] * q.L[i]; }
            var[en] = d / (s + 1e-20);
            lvl[en] = rms_of(q.L + SR, 3 * SR);
            rec_free(&q); api->destroy_instance(in);
        }
        CHECK(var[0] < 1e-6 && var[1] > 0.005, "L/R identici se off (%.1e), decorrelati se on (%.3f)", var[0], var[1]);
        CHECK(fabs(lvl[1] - lvl[0]) < 2.0, "ensemble non cambia il livello di piu' di 2 dB (off %.1f, on %.1f dBFS)", lvl[0], lvl[1]);

        /* Attack at 0 = immediate, Release at 0 = immediate */
        void *in = blank_inst();
        setp(in, "lvl_str", "0.8"); setp(in, "zone_str", "3"); setp(in, "att_str", "0"); setp(in, "release", "0");
        rec_t q; rec_init(&q, 2.0);
        ev_t e2[2] = { { 0.5, 1, 60, 100 }, { 1.0, 0, 60, 0 } };
        run(in, 2.0, e2, 2, 0, &q, NULL);
        double a_on  = rms_of(q.L + (int)(0.52 * SR), SR / 50);
        double a_off = rms_of(q.L + (int)(1.05 * SR), SR / 50);
        CHECK(a_on > -35.0 && a_off < -60.0, "Attack/Release a zero: suono entro 20 ms (%.1f dBFS), spento entro 50 ms (%.1f dBFS)", a_on, a_off);
        rec_free(&q); api->destroy_instance(in);
    }

    /* --------------------------------------------------------------- 11 */
    printf("\n11. Stato, parametri, JSON\n");
    {
        void *a = new_inst(6);
        setp(a, "tone", "0.77"); setp(a, "zone_voc", "Upper"); setp(a, "split", "55"); setp(a, "octave_transpose", "-1");
        setp(a, "ps_mode", "Manual"); setp(a, "velocity", "Dynamic"); setp(a, "hv_f4_up", "1");
        static char st[4096], cp[16384], ui[4096];
        int ls = api->get_param(a, "state", st, sizeof st);
        int lc = api->get_param(a, "chain_params", cp, sizeof cp);
        int lu = api->get_param(a, "ui_hierarchy", ui, sizeof ui);
        printf("      state %d byte, chain_params %d byte, ui_hierarchy %d byte\n", ls, lc, lu);
        CHECK(ls > 0 && lc > 0 && lu > 0, "get_param restituisce state / chain_params / ui_hierarchy");
        void *b = api->create_instance(".", NULL);
        setp(b, "state", st);
        static char st2[4096]; api->get_param(b, "state", st2, sizeof st2);
        CHECK(strcmp(st, st2) == 0, "round-trip dello stato identico");
        char v[32];
        api->get_param(a, "zone_voc", v, sizeof v);
        CHECK(strcmp(v, "2") == 0, "enum riportato come indice (zone_voc = %s)", v);
        FILE *f = fopen("/tmp/vdp33_chain_params.json", "w"); fputs(cp, f); fclose(f);
        f = fopen("/tmp/vdp33_ui_hierarchy.json", "w"); fputs(ui, f); fclose(f);
        api->destroy_instance(a); api->destroy_instance(b);
    }

    /* --------------------------------------------------------------- 12 */
    printf("\n12. CPU: 24 tasti + tutte le sezioni + vocoder + ensemble\n");
    {
        void *in = new_inst(preset_by_name("Full Mix"));
        setp(in, "lvl_str", "0.6"); setp(in, "lvl_voice", "0.6"); setp(in, "sibilance", "0.5");
        setp(in, "hv_m4_low", "1"); setp(in, "hv_f4_up", "1"); setp(in, "vib_depth", "0.3"); setp(in, "direct_mic", "0.2");
        for (int i = 0; i < 24; i++) send(in, 0x90, 36 + i * 2, 100);
        int16_t out[BLK * 2]; speech_t sp; memset(&sp, 0, sizeof sp); sp.cur = -1;
        int blocks = (60 * SR) / BLK;
        struct timespec t0, t1; clock_gettime(CLOCK_MONOTONIC, &t0);
        for (int b = 0; b < blocks; b++) {
            speech_block(&sp, (double)b * BLK / SR, (int16_t *)(g_mem + MOVE_AUDIO_IN_OFFSET), NULL);
            api->render_block(in, out, BLK);
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double el = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;
        printf("      60 s di audio in %.3f s  => %.2f%% di un core su questa macchina (%.1f us/blocco di 2902 us)\n",
               el, 100.0 * el / 60.0, 1e6 * el / blocks);
        CHECK(el / 60.0 < 0.10, "carico < 10%% di un core x86 (stima Move: ~3-5x, quindi ben sotto il budget)");
        for (int i = 0; i < 24; i++) send(in, 0x80, 36 + i * 2, 0);
        memset(g_mem + MOVE_AUDIO_IN_OFFSET, 0, BLK * 4);
        for (int b = 0; b < (12 * SR) / BLK; b++) api->render_block(in, out, BLK);
        clock_gettime(CLOCK_MONOTONIC, &t0);
        for (int b = 0; b < (10 * SR) / BLK; b++) api->render_block(in, out, BLK);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double el2 = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;
        printf("      silenzio dopo il rilascio: 10 s in %.4f s\n", el2);
        CHECK(el2 < 0.35 * (el / 6.0), "nessun rallentamento da denormali: a riposo costa meno di 1/3 del carico pieno (%.0f%%)", 100.0 * el2 / (el / 6.0));
        api->destroy_instance(in);
    }

    printf("\n%s (%d controlli falliti)\n", g_fail ? "ESITO: FALLITO" : "ESITO: TUTTO OK", g_fail);
    return g_fail ? 1 : 0;
}

// SPDX-License-Identifier: MIT
// 3DS-SDR - networked RTL-SDR scanner for New Nintendo 3DS

#include <3ds.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <malloc.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

// libctru linear heap reservation size; paired with linearSpaceFree() for diagnostics.
extern u32 __ctru_linear_heap_size;

#define AUDIO_RATE       48000
#define AUDIO_SAMPLES    1024
#define AUDIO_BUFS       8
#define SOC_ALIGN        0x1000
#define SOC_SIZE         0x100000
#define IQ_READ_BYTES    32768
#define WBFM_FIR_TAPS    121
#define NFM_IQ_FIR_TAPS   63
#define AUDIO_FIFO_SAMPLES 32768
// WFM keeps the public beta buffering unchanged for uninterrupted music/audio.
#define AUDIO_PREBUFFER_SAMPLES 12000  // WFM: 250 ms @ 48 kHz; near sync target
#define NDSP_TARGET_BUFS 4             // WFM: ~85 ms queued in NDSP
#define AUDIO_TARGET_SAMPLES 12288      // WFM: 256 ms, 37.5% of FIFO
// NFM is intentionally more responsive for scanner/voice use.
#define NFM_AUDIO_PREBUFFER_SAMPLES 3072 // 64 ms @ 48 kHz
#define NFM_NDSP_TARGET_BUFS 2           // ~43 ms queued in NDSP
#define NFM_AUDIO_TARGET_SAMPLES 4096    // ~85 ms software FIFO target
#define RESAMP_MIN_STEP 0.9950f         // +~5025 ppm output correction
#define RESAMP_MAX_STEP 1.0050f         // -~4975 ppm output correction
#define RESAMP_KP 0.0030f
#define RESAMP_KI 0.00012f
#define RESAMP_CONTROL_MS 100

// Raw rtl_tcp jitter buffer. 524288 bytes at 240 ksps (2 bytes/complex sample)
// gives about 1.09 seconds of elasticity.
#define IQ_FIFO_BYTES       1048576U
// WFM keeps the public beta jitter-buffer depth unchanged.
#define IQ_FIFO_START_BYTES  393216U   // WFM: 37.5%, ~819 ms @ 240 ksps
#define IQ_FIFO_TARGET_BYTES 393216U   // WFM: 37.5%
// NFM uses a shallower reservoir so carrier/scope/audio respond much sooner.
#define NFM_IQ_FIFO_START_BYTES   24000U // 50 ms startup @ 240 ksps (480 kB/s IQ)
#define NFM_IQ_FIFO_TARGET_BYTES  48000U // 100 ms live target
#define NFM_IQ_FIFO_MAX_BYTES     96000U // 200 ms hard latency ceiling
#define IQ_PACE_MIN          0.9950f
#define IQ_PACE_MAX          1.0050f
#define IQ_PACE_KP           0.0060f
#define IQ_PACE_KI           0.00010f
#define IQ_PACE_CONTROL_MS   100

#define FFT_N                 256
#define FFT_REFRESH_MS        40
#define FFT_DB_TOP            -10.0f
#define FFT_DB_BOTTOM         -50.0f
#define FFT_SMOOTH_ALPHA      0.18f

// Change these for your Archer C7 / rtl_tcp server.
#define DEFAULT_HOST     "10.0.0.15"
#define DEFAULT_PORT     1234
#define DEFAULT_FREQ_HZ  103300000U

typedef enum {
    MODE_NFM = 0,
    MODE_WBFM = 1
} RadioMode;

// Audio buffering policy follows the active demodulation mode.
static RadioMode g_audio_buffer_mode = MODE_WBFM;

typedef struct {
    int sock;
    RadioMode mode;
    u32 freq_hz;
    u32 sample_rate;
    int iq_decim;
    float fm_gain;
    float prev_i;
    float prev_q;
    int box_count;
    int box_i;
    int box_q;
    int audio_decim;
    int audio_count;
    float audio_accum;
    float deemph_y;
    float deemph_alpha;

    // WBFM mono audio FIR / decimator state.
    float wbfm_fir[WBFM_FIR_TAPS];
    int wbfm_fir_pos;
    int wbfm_decim_phase;

    // NFM complex channel filter / 5:1 decimator.  This replaces the old
    // 5-sample boxcar front-end with a real anti-alias/channel filter.
    float nfm_i_fir[NFM_IQ_FIR_TAPS];
    float nfm_q_fir[NFM_IQ_FIR_TAPS];
    int nfm_fir_pos;
    int nfm_decim_phase;

    // NFM voice conditioning at 48 kHz.
    float nfm_hp_x1;
    float nfm_hp_x2;
    float nfm_hp_y1;
    float nfm_hp_y2;
    float nfm_deemph_y;
    float nfm_lpf_y;

    bool have_i_byte;
    u8 i_byte;
    u32 recv_calls;
    u32 recv_bytes;
} Radio;


// ---------------------------------------------------------------------------
// v30 scanner core
// ---------------------------------------------------------------------------

#define SCAN_MAX_BANKS       12
#define SCAN_MAX_CHANNELS    48
#define SCAN_NAME_LEN        24
#define SCAN_SETTLE_MS       120
#define SCAN_CHECK_MS        220
#define SCAN_OPEN_MS         25
#define SCAN_CLOSE_MS        120
#define SCAN_TIME_HOLD_MS    5000
#define SCAN_CFG_PATH        "sdmc:/3ds/3ds-sdr/scanner-v21.cfg"
#define SERVER_CFG_PATH      "sdmc:/3ds/3ds-sdr/server.cfg"

typedef enum {
    SCAN_RESUME_DELAY = 0,
    SCAN_RESUME_CARRIER,
    SCAN_RESUME_TIME
} ScanResumeMode;

typedef enum {
    SCANNER_STOPPED = 0,
    SCANNER_SETTLE,
    SCANNER_CHECK,
    SCANNER_RECEIVE,
    SCANNER_DELAY
} ScannerState;

typedef enum {
    SCANNER_RUN_HOLD = 0,
    SCANNER_RUN_SCAN
} ScannerRunMode;

typedef struct {
    char name[SCAN_NAME_LEN];
    u32 freq_hz;
    RadioMode mode;
    bool enabled;
    bool temp_avoid;
    bool priority;
    u32 delay_ms;
    int step_index;
    int squelch_override;   // -1 = use global squelch
    u32 hit_count;
} ScanChannel;

typedef struct {
    char name[SCAN_NAME_LEN];
    bool enabled;
    int channel_count;
    ScanChannel channels[SCAN_MAX_CHANNELS];
} ScanBank;

typedef struct {
    ScanBank banks[SCAN_MAX_BANKS];
    int bank_count;
    int bank_index;
    int channel_index;
    ScannerState state;
    ScanResumeMode resume_mode;
    ScannerRunMode run_mode;
    bool scanning; /* compatibility mirror of run_mode == SCANNER_RUN_SCAN */
    int sql_level;              // 0..100
    bool squelch_open;
    u64 state_since_ms;
    u64 sql_candidate_ms;
    bool sql_candidate_open;
    float rf_db;
    float noise_db;
    float margin_db;
    float open_margin_db;
    float close_margin_db;
    u32 hits;
    u32 passes;

    // Firmware/UI state
    bool scope_visible;
    int scope_screen;       // 0 = bottom, 1 = top
    int scope_overlay;      // 0 = off, 1 = minimal, 2 = full
    bool hold_audio_monitor; // false=squelched, true=forced open while HOLD
    bool vfo_mode;
    bool monitor;
    u32 vfo_freq_hz;
    RadioMode vfo_radio_mode;
    int vfo_step_index;
    u32 time_hold_ms;
} Scanner;

static Scanner g_scan;
static bool g_audio_gate_open = true;

/* Master receiver transport/DSP pause. Scanner run state is preserved while
 * paused; rtl_tcp is still drained so resume always starts from live IQ. */
static bool g_receiver_running = true;
static u64 g_receiver_paused_at_ms = 0;
static u64 g_receiver_resuming_until_ms = 0;

/* Cached PTMU battery state for the top-banner indicator. */
static bool g_ptmu_ready = false;
static u8 g_battery_level = 0;
static u8 g_battery_charging = 0;
static u64 g_battery_last_poll_ms = 0;

/*
 * Channel-selective squelch detector.
 *
 * The old detector measured total power across the complete rtl_tcp bandwidth,
 * so an off-frequency signal could open squelch.  These values are fed from
 * the paced DSP stream after the NFM front-end decimation.
 */
static float g_sql_dc_i = 0.0f;
static float g_sql_dc_q = 0.0f;
static float g_sql_lp_i = 0.0f;
static float g_sql_lp_q = 0.0f;
static float g_sql_center_power = 1.0e-9f;
static float g_sql_adj_power = 1.0e-9f;
static bool g_sql_metric_valid = false;
static u32 g_sql_metric_samples = 0;

/*
 * Low-latency NFM scan detector.  The normal squelch metric above follows the
 * paced/jitter-buffered DSP stream, which is ideal for stable playback but can
 * be too far behind live RF for scanner channel decisions.  While scanning NFM
 * channels, this second detector is fed directly from newly received rtl_tcp IQ
 * after the retune discard window.  It is used only to decide whether SCAN
 * should stop on the current channel; demod/audio/scope continue to use the
 * regular buffered DSP path.
 */
static float g_scanfast_dc_i = 0.0f;
static float g_scanfast_dc_q = 0.0f;
static float g_scanfast_lp_i = 0.0f;
static float g_scanfast_lp_q = 0.0f;
static float g_scanfast_center_power = 1.0e-9f;
static float g_scanfast_adj_power = 1.0e-9f;
static bool g_scanfast_metric_valid = false;
static u32 g_scanfast_metric_samples = 0;
static bool g_scanfast_candidate_open = false;
static u64 g_scanfast_candidate_ms = 0;
static bool g_scanfast_have_i = false;
static u8 g_scanfast_i_byte = 0;

static bool g_scope_allowed = true;
static bool g_ui_dirty = true;
static u64 g_last_ui_render_ms = 0;
#define TEXT_UI_REFRESH_MS 200

static const u32 g_step_hz[] = {
    1000U, 2500U, 5000U, 6250U, 10000U, 12500U,
    15000U, 20000U, 25000U, 50000U, 100000U
};
#define STEP_COUNT ((int)(sizeof(g_step_hz)/sizeof(g_step_hz[0])))

typedef enum { MENU_CHANNEL=0, MENU_BANKS, MENU_SCAN, MENU_RADIO, MENU_DISPLAY, MENU_SYSTEM, MENU_CAT_COUNT } MenuCategory;
static int menu_item_count(MenuCategory c);
static const char *menu_cat_name(MenuCategory c);
static void menu_item_text(MenuCategory cat,int item,char *buf,size_t n);
static void radio_retune_reset(Radio *r);
static void tune_frequency(Radio *r,u32 freq,RadioMode mode);
static bool radio_reconnect(Radio *r);
static bool g_menu_open = false;
static MenuCategory g_menu_cat = MENU_CHANNEL;
static int g_menu_item = 0;
static bool g_show_debug = false;
static bool g_input_lock_until_release = false;

/* Menu memory browser is intentionally independent from the live receiver. */
static int g_edit_bank_index = 0;
static int g_edit_channel_index = 0;

/* Non-blocking frequency editor: DSP/audio continue in the main loop. */
static bool g_freq_edit_active = false;
static u32 *g_freq_edit_target = NULL;
static u32 g_freq_edit_original = 0;
static int g_freq_edit_digit = 3;
static bool g_freq_edit_live_vfo = false;
static char g_freq_edit_title[40] = {0};

/* After a retune/keyboard return, incoming socket IQ is discarded briefly so
 * pre-change network backlog can never enter the new DSP pipeline. */
static u64 g_rx_discard_until_ms = 0;
#define RX_RETUNE_DISCARD_MS 45
#define NFM_RX_RETUNE_DISCARD_MS 250
static u32 g_rtl_cmd_failures = 0;

/* Confirmed rtl_tcp server for this session.  Loaded from a tiny separate
 * settings file so scanner-v21.cfg remains fully backward-compatible. */
static char g_server_host[16] = DEFAULT_HOST;



static inline void scanner_ui_dirty(void)
{
    g_ui_dirty = true;
}

static inline bool scanner_is_scanning(const Scanner *s)
{
    return s->run_mode == SCANNER_RUN_SCAN;
}

static inline bool scanner_is_hold(const Scanner *s)
{
    return s->run_mode == SCANNER_RUN_HOLD;
}

static const char *scan_resume_name(ScanResumeMode m)
{
    switch (m) {
        case SCAN_RESUME_CARRIER: return "CARRIER";
        case SCAN_RESUME_TIME:    return "TIME";
        default:                  return "DELAY";
    }
}

static const char *mode_name(RadioMode m)
{
    return m == MODE_NFM ? "NFM" : "WFM";
}


static const char *scanner_state_name(ScannerState s)
{
    switch (s) {
        case SCANNER_SETTLE:  return "SETTLE";
        case SCANNER_CHECK:   return "CHECK";
        case SCANNER_RECEIVE: return "RECEIVE";
        case SCANNER_DELAY:   return "DELAY";
        default:              return "STOP";
    }
}

static ScanBank *scanner_bank(Scanner *s)
{
    if (s->bank_count <= 0) return NULL;
    if (s->bank_index < 0) s->bank_index = 0;
    if (s->bank_index >= s->bank_count) s->bank_index = s->bank_count - 1;
    return &s->banks[s->bank_index];
}

static ScanChannel *scanner_channel(Scanner *s)
{
    ScanBank *b = scanner_bank(s);
    if (!b || b->channel_count <= 0) return NULL;
    if (s->channel_index < 0) s->channel_index = 0;
    if (s->channel_index >= b->channel_count) s->channel_index = b->channel_count - 1;
    return &b->channels[s->channel_index];
}

static ScanBank *edit_bank(void)
{
    if(g_scan.bank_count<=0) return NULL;
    if(g_edit_bank_index<0) g_edit_bank_index=0;
    if(g_edit_bank_index>=g_scan.bank_count) g_edit_bank_index=g_scan.bank_count-1;
    return &g_scan.banks[g_edit_bank_index];
}

static ScanChannel *edit_channel(void)
{
    ScanBank *b=edit_bank();
    if(!b || b->channel_count<=0) return NULL;
    if(g_edit_channel_index<0) g_edit_channel_index=0;
    if(g_edit_channel_index>=b->channel_count) g_edit_channel_index=b->channel_count-1;
    return &b->channels[g_edit_channel_index];
}

static void edit_cursor_from_live(void)
{
    g_edit_bank_index=g_scan.bank_index;
    g_edit_channel_index=g_scan.channel_index;
    (void)edit_channel();
}

static void edit_cycle_bank(int dir)
{
    if(g_scan.bank_count<=0) return;
    g_edit_bank_index+=dir;
    if(g_edit_bank_index<0) g_edit_bank_index=g_scan.bank_count-1;
    if(g_edit_bank_index>=g_scan.bank_count) g_edit_bank_index=0;
    ScanBank *b=edit_bank();
    if(!b || b->channel_count<=0) g_edit_channel_index=0;
    else if(g_edit_channel_index>=b->channel_count) g_edit_channel_index=0;
    scanner_ui_dirty();
}

static void edit_cycle_channel(int dir)
{
    ScanBank *b=edit_bank();
    if(!b || b->channel_count<=0) return;
    g_edit_channel_index+=dir;
    if(g_edit_channel_index<0) g_edit_channel_index=b->channel_count-1;
    if(g_edit_channel_index>=b->channel_count) g_edit_channel_index=0;
    scanner_ui_dirty();
}

static void scanner_defaults(Scanner *s)
{
    memset(s, 0, sizeof(*s));
    s->bank_count = 3;
    s->resume_mode = SCAN_RESUME_DELAY;
    s->sql_level = 35;
    s->noise_db = -42.0f;
    s->rf_db = -42.0f;
    s->run_mode = SCANNER_RUN_HOLD;
    s->scanning = false;
    s->state = SCANNER_STOPPED;
    s->scope_visible = false;
    s->scope_screen = 0;
    s->scope_overlay = 1;
    s->hold_audio_monitor = false;
    s->vfo_mode = false;
    s->vfo_freq_hz = DEFAULT_FREQ_HZ;
    s->vfo_radio_mode = MODE_WBFM;
    s->vfo_step_index = 10;
    s->time_hold_ms = SCAN_TIME_HOLD_MS;

    ScanBank *b = &s->banks[0];
    snprintf(b->name, sizeof(b->name), "Amateur");
    b->enabled = true;
    b->channel_count = 2;
    snprintf(b->channels[0].name, sizeof(b->channels[0].name), "2m Simplex");
    b->channels[0].freq_hz = 146520000U;
    b->channels[0].mode = MODE_NFM;
    b->channels[0].enabled = true;
    b->channels[0].delay_ms = 2000;
    b->channels[0].step_index = 5; b->channels[0].squelch_override = -1;
    snprintf(b->channels[1].name, sizeof(b->channels[1].name), "70cm Simplex");
    b->channels[1].freq_hz = 446000000U;
    b->channels[1].mode = MODE_NFM;
    b->channels[1].enabled = true;
    b->channels[1].delay_ms = 2000;
    b->channels[1].step_index = 5; b->channels[1].squelch_override = -1;

    b = &s->banks[1];
    snprintf(b->name, sizeof(b->name), "Weather");
    b->enabled = true;
    b->channel_count = 1;
    snprintf(b->channels[0].name, sizeof(b->channels[0].name), "NOAA");
    b->channels[0].freq_hz = 162550000U;
    b->channels[0].mode = MODE_NFM;
    b->channels[0].enabled = true;
    b->channels[0].delay_ms = 2000;
    b->channels[0].step_index = 4; b->channels[0].squelch_override = -1;

    b = &s->banks[2];
    snprintf(b->name, sizeof(b->name), "Broadcast FM");
    b->enabled = false;
    b->channel_count = 1;
    snprintf(b->channels[0].name, sizeof(b->channels[0].name), "FM 103.3");
    b->channels[0].freq_hz = 103300000U;
    b->channels[0].mode = MODE_WBFM;
    b->channels[0].enabled = true;
    b->channels[0].delay_ms = 2000;
    b->channels[0].step_index = 10; b->channels[0].squelch_override = -1;
}

static void scanner_save(const Scanner *s)
{
    mkdir("sdmc:/3ds", 0777);
    mkdir("sdmc:/3ds/3ds-sdr", 0777);
    FILE *f = fopen(SCAN_CFG_PATH, "w");
    if (!f) return;

    fprintf(f, "V21|%d|%d|%d|%u|%u|%d|%d|%d|%d|%d\n",
            (int)s->resume_mode, s->sql_level, 0, /* legacy scope-on-hit slot */
            (unsigned)s->time_hold_ms, (unsigned)s->vfo_freq_hz,
            (int)s->vfo_radio_mode, s->vfo_step_index,
            s->scope_screen, s->scope_overlay,
            s->hold_audio_monitor?1:0);
    for (int bi=0; bi<s->bank_count; bi++) {
        const ScanBank *b=&s->banks[bi];
        fprintf(f,"B|%d|%s\n",b->enabled?1:0,b->name);
        for (int ci=0; ci<b->channel_count; ci++) {
            const ScanChannel *c=&b->channels[ci];
            fprintf(f,"C|%u|%d|%d|%d|%u|%d|%d|%u|%s\n",
                    (unsigned)c->freq_hz,(int)c->mode,c->enabled?1:0,c->priority?1:0,
                    (unsigned)c->delay_ms,c->step_index,c->squelch_override,
                    (unsigned)c->hit_count,c->name);
        }
        fprintf(f,"E\n");
    }
    fclose(f);
}

static void trim_eol(char *s)
{
    size_t n = strlen(s);
    while (n && (s[n-1] == '\n' || s[n-1] == '\r')) s[--n] = 0;
}

static void server_host_load(void)
{
    snprintf(g_server_host,sizeof(g_server_host),"%s",DEFAULT_HOST);
    FILE *f=fopen(SERVER_CFG_PATH,"r");
    if(!f) return;

    char line[64]={0};
    if(fgets(line,sizeof(line),f)){
        trim_eol(line);
        const char *ip=!strncmp(line,"IP|",3)?line+3:line;
        struct in_addr tmp;
        if(inet_aton(ip,&tmp)!=0)
            snprintf(g_server_host,sizeof(g_server_host),"%s",ip);
    }
    fclose(f);
}

static void server_host_save(void)
{
    mkdir("sdmc:/3ds",0777);
    mkdir("sdmc:/3ds/3ds-sdr",0777);
    FILE *f=fopen(SERVER_CFG_PATH,"w");
    if(!f) return;
    fprintf(f,"IP|%s\n",g_server_host);
    fclose(f);
}

static void scanner_load(Scanner *s)
{
    scanner_defaults(s);
    FILE *f=fopen(SCAN_CFG_PATH,"r");
    if (!f) return;

    Scanner tmp; scanner_defaults(&tmp);
    tmp.bank_count=0;
    char line[192]; int bi=-1;
    while (fgets(line,sizeof(line),f)) {
        trim_eol(line);
        if (!strncmp(line,"V21|",4)) {
            int rm=0,sql=35,scope=0,vm=1,vsi=10,sscreen=0,soverlay=1,holdmon=0;
            unsigned th=SCAN_TIME_HOLD_MS,vf=DEFAULT_FREQ_HZ;
            if (sscanf(line+4,"%d|%d|%d|%u|%u|%d|%d|%d|%d|%d",
                       &rm,&sql,&scope,&th,&vf,&vm,&vsi,&sscreen,&soverlay,&holdmon)>=4) {
                tmp.resume_mode=(rm<0||rm>2)?SCAN_RESUME_DELAY:(ScanResumeMode)rm;
                tmp.sql_level=sql<0?0:(sql>100?100:sql);
                (void)scope; /* legacy scope-on-hit field: accepted but ignored */
                tmp.time_hold_ms=th; tmp.vfo_freq_hz=vf;
                tmp.vfo_radio_mode=vm==MODE_NFM?MODE_NFM:MODE_WBFM;
                tmp.vfo_step_index=vsi<0?0:(vsi>=STEP_COUNT?STEP_COUNT-1:vsi);
                tmp.scope_screen=(sscreen==1)?1:0;
                tmp.scope_overlay=soverlay<0?0:(soverlay>2?2:soverlay);
                tmp.hold_audio_monitor=holdmon!=0;
            }
        } else if (!strncmp(line,"B|",2)) {
            if (tmp.bank_count>=SCAN_MAX_BANKS) continue;
            bi=tmp.bank_count++; int en=1; char name[SCAN_NAME_LEN]={0};
            ScanBank *b=&tmp.banks[bi];
            memset(b,0,sizeof(*b));
            sscanf(line+2,"%d|%23[^\n]",&en,name);
            b->enabled=en!=0;
            snprintf(b->name,sizeof(b->name),"%s",name[0]?name:"Bank");
        } else if (!strcmp(line,"E")) bi=-1;
        else if (!strncmp(line,"C|",2) && bi>=0) {
            ScanBank *b=&tmp.banks[bi]; if (b->channel_count>=SCAN_MAX_CHANNELS) continue;
            unsigned fq=0,delay=2000,hits=0; int mode=0,en=1,pri=0,step=5,sq=-1; char name[SCAN_NAME_LEN]={0};
            int n=sscanf(line+2,"%u|%d|%d|%d|%u|%d|%d|%u|%23[^\n]",&fq,&mode,&en,&pri,&delay,&step,&sq,&hits,name);
            if (n>=7) {
                ScanChannel *c=&b->channels[b->channel_count++]; memset(c,0,sizeof(*c));
                c->freq_hz=fq; c->mode=mode==MODE_WBFM?MODE_WBFM:MODE_NFM; c->enabled=en!=0; c->priority=pri!=0;
                c->delay_ms=delay; c->step_index=step<0?0:(step>=STEP_COUNT?STEP_COUNT-1:step); c->squelch_override=sq;
                c->hit_count=hits; snprintf(c->name,sizeof(c->name),"%s",name[0]?name:"Channel");
            }
        }
    }
    fclose(f);
    if (tmp.bank_count>0) *s=tmp;
    s->scanning=false; s->state=SCANNER_STOPPED; s->scope_visible=false; s->vfo_mode=false;
}

static bool scanner_channel_eligible(const Scanner *s, int bi, int ci)
{
    if (bi < 0 || bi >= s->bank_count) return false;
    const ScanBank *b = &s->banks[bi];
    if (!b->enabled || ci < 0 || ci >= b->channel_count) return false;
    const ScanChannel *c = &b->channels[ci];
    return c->enabled && !c->temp_avoid;
}

static int scanner_eligible_count(const Scanner *s)
{
    int count=0;
    for(int bi=0;bi<s->bank_count;bi++){
        const ScanBank *b=&s->banks[bi];
        if(!b->enabled) continue;
        for(int ci=0;ci<b->channel_count;ci++)
            if(scanner_channel_eligible(s,bi,ci))
                count++;
    }
    return count;
}

static int scanner_total_channel_count(const Scanner *s)
{
    int count=0;
    for(int bi=0;bi<s->bank_count;bi++)
        count+=s->banks[bi].channel_count;
    return count;
}

static int scanner_bank_eligible_count(const Scanner *s,int bi)
{
    if(bi<0 || bi>=s->bank_count) return 0;
    int count=0;
    const ScanBank *b=&s->banks[bi];
    for(int ci=0;ci<b->channel_count;ci++)
        if(scanner_channel_eligible(s,bi,ci))
            count++;
    return count;
}

static bool scanner_advance(Scanner *s,int dir)
{
    if(s->bank_count<=0) return false;

    /*
     * Walk the live memory database explicitly instead of relying on channel
     * counts during wraparound. This makes dynamic banks/channels behave the
     * same as memories loaded at startup.
     */
    int total_slots=0;
    for(int bi=0;bi<s->bank_count;bi++)
        total_slots+=s->banks[bi].channel_count;

    if(total_slots<=0 || scanner_eligible_count(s)<=0){
        s->scanning=false;
        s->state=SCANNER_STOPPED;
        g_audio_gate_open=false;
        g_scope_allowed=false;
        scanner_ui_dirty();
        return false;
    }

    int start_b=s->bank_index;
    int start_c=s->channel_index;
    int bi=start_b;
    int ci=start_c;

    for(int visited=0;visited<total_slots;visited++){
        if(dir>=0){
            /* Advance one physical memory slot. Empty banks are skipped. */
            do {
                ci++;
                if(bi<0 || bi>=s->bank_count){ bi=0; ci=0; }

                if(ci>=s->banks[bi].channel_count){
                    bi=(bi+1)%s->bank_count;
                    ci=0;
                }
            } while(s->banks[bi].channel_count<=0);
        }else{
            do {
                ci--;
                if(ci<0){
                    bi--;
                    if(bi<0) bi=s->bank_count-1;
                    ci=s->banks[bi].channel_count-1;
                }
            } while(s->banks[bi].channel_count<=0);
        }

        if(scanner_channel_eligible(s,bi,ci)){
            s->bank_index=bi;
            s->channel_index=ci;
            scanner_ui_dirty();
            return true;
        }
    }

    /* There were physical memories, but none currently eligible. */
    s->run_mode=SCANNER_RUN_HOLD;
    s->scanning=false;
    s->state=SCANNER_STOPPED;
    g_audio_gate_open=false;
    g_scope_allowed=false;
    scanner_ui_dirty();
    return false;
}

static void scanner_tune_current(Scanner *s, Radio *r);

static bool scanner_select_bank(Scanner *s, Radio *r, int dir)
{
    if (s->bank_count <= 0) return false;
    int bi=s->bank_index+dir;
    if (bi < 0) bi = s->bank_count - 1;
    if (bi >= s->bank_count) bi = 0;
    s->bank_index=bi; ScanBank *b=&s->banks[bi];
    if(b->channel_count>0){ if(s->channel_index>=b->channel_count)s->channel_index=0; ScanChannel *c=&b->channels[s->channel_index]; scanner_tune_current(s,r); (void)c; }
    else s->channel_index=0;
    scanner_ui_dirty(); return true;
}

static bool scanner_select_channel_in_bank(Scanner *s, Radio *r, int dir)
{
    ScanBank *b=scanner_bank(s); if(!b||b->channel_count<=0)return false;
    int ci=s->channel_index+dir; if(ci<0)ci=b->channel_count-1; if(ci>=b->channel_count)ci=0;
    s->channel_index=ci; scanner_tune_current(s,r); scanner_ui_dirty(); return true;
}

static inline void scanner_channel_metric_sample(RadioMode mode,float i,float q)
{
    /*
     * Remove only very slow I/Q DC. RTL dongles commonly have a center spike;
     * allowing it into the squelch detector would make "exactly centered"
     * appear busy even when there is no radio signal.
     *
     * NFM metrics are intentionally sampled from the raw 240 ksps IQ stream
     * before the 63-tap channel FIR.  The coefficients below are therefore
     * scaled for 240 ksps so the detector keeps approximately the same time
     * constants it previously had at 48 ksps.  Measuring after the NFM FIR
     * makes the rejected/adjacent-noise term artificially tiny and can hold
     * squelch open on an otherwise idle simplex channel.
     */
    const float dc_a = (mode==MODE_NFM) ? 0.00016f : 0.00025f;
    g_sql_dc_i += dc_a*(i-g_sql_dc_i);
    g_sql_dc_q += dc_a*(q-g_sql_dc_q);
    float x=i-g_sql_dc_i;
    float y=q-g_sql_dc_q;

    /*
     * Complex low-pass around zero Hz.  NFM uses the raw 240 ksps stream here;
     * 0.204 is the 240 ksps equivalent of the old 0.68 coefficient at 48 ksps
     * (about the same ~8.7 kHz one-pole corner).  The residual outside this
     * center estimate remains available as a meaningful adjacent-noise term.
     * WFM keeps its existing wider detector coefficient unchanged.
     */
    const float lp_a = (mode==MODE_NFM) ? 0.204f : 0.93f;
    g_sql_lp_i += lp_a*(x-g_sql_lp_i);
    g_sql_lp_q += lp_a*(y-g_sql_lp_q);

    float ri=x-g_sql_lp_i;
    float rq=y-g_sql_lp_q;
    float center=g_sql_lp_i*g_sql_lp_i + g_sql_lp_q*g_sql_lp_q;
    float adjacent=ri*ri + rq*rq;

    /* About a few milliseconds of smoothing at either DSP rate. */
    const float pa = (mode==MODE_NFM) ? 0.00363f : 0.004f;
    g_sql_center_power += pa*(center-g_sql_center_power);
    g_sql_adj_power += pa*(adjacent-g_sql_adj_power);

    if(g_sql_metric_samples<1000000U) g_sql_metric_samples++;
    if(g_sql_metric_samples >= 960U)
        g_sql_metric_valid=true;
}

static int scanner_effective_sql(Scanner *s, bool *has_override);

static inline void scanner_scanfast_metric_reset(void)
{
    g_scanfast_dc_i=0.0f;
    g_scanfast_dc_q=0.0f;
    g_scanfast_lp_i=0.0f;
    g_scanfast_lp_q=0.0f;
    g_scanfast_center_power=1.0e-9f;
    g_scanfast_adj_power=1.0e-9f;
    g_scanfast_metric_valid=false;
    g_scanfast_metric_samples=0;
    g_scanfast_candidate_open=false;
    g_scanfast_candidate_ms=osGetTime();
    g_scanfast_have_i=false;
    g_scanfast_i_byte=0;
}

static inline void scanner_scanfast_metric_pair(u8 ib,u8 qb)
{
    float i=((float)ib-128.0f)*(1.0f/128.0f);
    float q=((float)qb-128.0f)*(1.0f/128.0f);

    /* dev5 diagnostic: do NOT adaptively subtract the centered I/Q vector
     * in the live NFM scan detector. A correctly tuned unmodulated FM carrier
     * appears close to DC at complex baseband, so the former ~26 ms DC
     * estimator could learn and cancel the very carrier SCAN was trying to
     * track. The normal buffered DSP/squelch path is intentionally untouched. */
    float x=i;
    float y=q;

    const float lp_a=0.204f;
    g_scanfast_lp_i += lp_a*(x-g_scanfast_lp_i);
    g_scanfast_lp_q += lp_a*(y-g_scanfast_lp_q);

    float ri=x-g_scanfast_lp_i;
    float rq=y-g_scanfast_lp_q;
    float center=g_scanfast_lp_i*g_scanfast_lp_i + g_scanfast_lp_q*g_scanfast_lp_q;
    float adjacent=ri*ri + rq*rq;

    const float pa=0.00363f;
    g_scanfast_center_power += pa*(center-g_scanfast_center_power);
    g_scanfast_adj_power += pa*(adjacent-g_scanfast_adj_power);

    if(g_scanfast_metric_samples<1000000U) g_scanfast_metric_samples++;
    if(g_scanfast_metric_samples>=960U) g_scanfast_metric_valid=true;
}

static void scanner_scanfast_feed(const u8 *buf,u32 bytes)
{
    u32 n=0;
    if(g_scanfast_have_i && bytes>0){
        scanner_scanfast_metric_pair(g_scanfast_i_byte,buf[0]);
        g_scanfast_have_i=false;
        n=1;
    }
    for(;n+1<bytes;n+=2)
        scanner_scanfast_metric_pair(buf[n],buf[n+1]);
    if(n<bytes){
        g_scanfast_i_byte=buf[n];
        g_scanfast_have_i=true;
    }
}

static void scanner_scanfast_update_squelch(Scanner *s,u64 now)
{
    if(!g_scanfast_metric_valid) return;

    float center_db=10.0f*log10f(g_scanfast_center_power+1.0e-12f);
    float adj_db=10.0f*log10f(g_scanfast_adj_power+1.0e-12f);
    float margin_db=center_db-adj_db;

    int effSql=scanner_effective_sql(s,NULL);
    float open_margin_db=-1.5f + (float)effSql*0.195f;
    float close_margin_db=open_margin_db-2.5f;

    /* Once NFM SCAN is active, this fresh-IQ detector owns carrier state for
     * the complete CHECK -> RECEIVE -> DELAY lifecycle.  Use the same
     * hysteresis/debounce policy as normal squelch so a brief fade cannot
     * immediately kick the scanner off an active simplex transmission. */
    bool want_open=s->squelch_open
        ? (margin_db>=close_margin_db)
        : (margin_db>=open_margin_db);

    if(want_open!=g_scanfast_candidate_open){
        g_scanfast_candidate_open=want_open;
        g_scanfast_candidate_ms=now;
    }

    u64 need=want_open?SCAN_OPEN_MS:SCAN_CLOSE_MS;
    if(want_open!=s->squelch_open &&
       now-g_scanfast_candidate_ms>=need){
        bool was_open=s->squelch_open;
        s->squelch_open=want_open;
        s->sql_candidate_open=want_open;
        s->sql_candidate_ms=now;
        s->rf_db=center_db;
        s->noise_db=adj_db;
        s->margin_db=margin_db;
        s->open_margin_db=open_margin_db;
        s->close_margin_db=close_margin_db;
        if(want_open && !was_open){
            s->hits++;
            ScanChannel *hc=scanner_channel(s);
            if(!s->vfo_mode && hc) hc->hit_count++;
        }
        scanner_ui_dirty();
    }else{
        /* Keep live detector telemetry current while staying in the same state. */
        s->rf_db=center_db;
        s->noise_db=adj_db;
        s->margin_db=margin_db;
        s->open_margin_db=open_margin_db;
        s->close_margin_db=close_margin_db;
    }
}

static inline void scanner_channel_metric_reset(void)
{
    g_sql_dc_i=0.0f;
    g_sql_dc_q=0.0f;
    g_sql_lp_i=0.0f;
    g_sql_lp_q=0.0f;
    g_sql_center_power=1.0e-9f;
    g_sql_adj_power=1.0e-9f;
    g_sql_metric_valid=false;
    g_sql_metric_samples=0;
}

static int scanner_effective_sql(Scanner *s, bool *has_override)
{
    int effSql=s->sql_level;
    bool ovr=false;
    ScanChannel *sqc=scanner_channel(s);
    if(!s->vfo_mode && sqc && sqc->squelch_override>=0){
        effSql=sqc->squelch_override;
        ovr=true;
    }
    if(has_override) *has_override=ovr;
    return effSql;
}

static void scanner_update_squelch(Scanner *s,u64 now)
{
    if(!g_sql_metric_valid) return;

    /*
     * RF now means energy inside the selected center channel; NF is energy
     * rejected outside that channel. Their difference is a gain-independent
     * center-channel quality margin, so AGC and nearby signals no longer look
     * like a valid centered carrier merely because total RF power is high.
     */
    float center_db=10.0f*log10f(g_sql_center_power+1.0e-12f);
    float adj_db=10.0f*log10f(g_sql_adj_power+1.0e-12f);

    s->rf_db += 0.28f*(center_db-s->rf_db);
    s->noise_db += 0.28f*(adj_db-s->noise_db);
    s->margin_db=s->rf_db-s->noise_db;

    int effSql=scanner_effective_sql(s,NULL);

    /*
     * SQL 0..100 maps to a center-vs-adjacent quality requirement.
     * 0 is permissive; 100 is intentionally very strict.
     */
    s->open_margin_db=-1.5f + (float)effSql*0.195f; /* -1.5 .. 18 dB */
    s->close_margin_db=s->open_margin_db-2.5f;      /* hysteresis */

    bool want_open=s->squelch_open
        ? (s->margin_db>=s->close_margin_db)
        : (s->margin_db>=s->open_margin_db);

    if(want_open!=s->sql_candidate_open){
        s->sql_candidate_open=want_open;
        s->sql_candidate_ms=now;
    }

    u64 need=want_open?SCAN_OPEN_MS:SCAN_CLOSE_MS;
    if(want_open!=s->squelch_open && now-s->sql_candidate_ms>=need){
        s->squelch_open=want_open;
        if(want_open){
            s->hits++;
            ScanChannel *hc=scanner_channel(s);
            if(!s->vfo_mode && hc) hc->hit_count++;
        }
        scanner_ui_dirty();
    }
}


static ndspWaveBuf waveBuf[AUDIO_BUFS];
static s16 *audioMem = NULL;
static int audioWrite = 0;

// DSP -> NDSP producer/consumer FIFO.
// 32768 samples at 48 kHz = ~683 ms of elasticity.
// v11 prebuffers near the clock-sync target and adaptively bridges SDR/NDSP clocks.
static s16 audioFifo[AUDIO_FIFO_SAMPLES];
static volatile u32 audioFifoRead = 0;
static volatile u32 audioFifoWrite = 0;
static volatile u32 audioFifoCount = 0;
static u32 g_audio_overruns = 0;
static u32 g_audio_rebuffers = 0;
static u64 g_audio_starved_ms = 0;
static u64 g_starve_start_ms = 0;
static bool g_audio_started_once = false;
static bool g_audio_rebuffering = true;

// Adaptive audio clock bridge. The SDR and 3DS audio hardware have independent
// oscillators, so "240 kHz / 5 == 48 kHz" is only nominally true forever.
static float g_resamp_step = 1.0f;       // input samples per output sample
static float g_resamp_integral = 0.0f;
static float g_resamp_phase = 0.0f;
static float g_resamp_prev = 0.0f;
static bool g_resamp_have_prev = false;
static u64 g_resamp_last_control_ms = 0;
static u64 g_pcm_source_samples = 0;
static u64 g_pcm_output_samples = 0;

// Per-status-window FIFO excursion diagnostics.
static u32 g_fifo_window_min = AUDIO_FIFO_SAMPLES;
static u32 g_fifo_window_max = 0;

// rtl_tcp -> DSP jitter buffer.
static u8 *iqFifo = NULL;
static u32 iqFifoRead = 0;
static u32 iqFifoWrite = 0;
static u32 iqFifoCount = 0;
static u32 g_iq_overruns = 0;
static u64 g_iq_latency_drops = 0;
static u64 g_iq_accepted_bytes = 0;
static u64 g_iq_dsp_pairs = 0;
static bool g_iq_started = false;

static double g_iq_credit = 0.0;
static u64 g_iq_last_pace_ms = 0;
static float g_iq_pace = 1.0f;
static float g_iq_pace_integral = 0.0f;
static u64 g_iq_last_control_ms = 0;

static u32 g_iq_window_min = IQ_FIFO_BYTES;
static u32 g_iq_window_max = 0;

// Lightweight spectrum display. 256-point FFT at ~10 Hz.
static float fftRe[FFT_N];
static float fftIm[FFT_N];
static float fftMag[FFT_N];
static float fftSmooth[FFT_N];
static bool fftSmoothInit = false;
static u32 fftFill = 0;
static bool fftReady = false;
static bool fftRetunePending = false;

/* X-button display cycle: normal -> scope -> waterfall -> normal.
 * Scope and waterfall share the same FFT + smoothing pipeline. */
static bool g_waterfall_visible = false;
#define WATERFALL_ROWS 240
static u8 g_waterfall[WATERFALL_ROWS][FFT_N];
static int g_waterfall_head = 0;
static int g_waterfall_count = 0;

static u32 *socBuffer = NULL;

// 121-tap Kaiser-windowed low-pass for 240 kHz -> 48 kHz WBFM mono.
// NFM complex-IQ low-pass for 240 ksps -> 48 ksps decimation.
// 63-tap Hamming-windowed LPF, ~12 kHz cutoff.  This keeps normal 5 kHz
// deviation voice comfortably inside the channel while rejecting energy that
// would otherwise alias through the old boxcar decimator.
static const float nfm_iq_lpf[NFM_IQ_FIR_TAPS] = {
    -0.0002534599f, 0.0000000000f, 0.0003028303f, 0.0006745172f,
     0.0011169001f, 0.0016020742f, 0.0020652623f, 0.0024052586f,
     0.0024939045f, 0.0021945208f, 0.0013875383f,-0.0000000000f,
    -0.0019655530f,-0.0044078514f,-0.0071141381f,-0.0097616817f,
    -0.0119356508f,-0.0131632271f,-0.0129614398f,-0.0108939528f,
    -0.0066302876f, 0.0000000000f, 0.0089656637f, 0.0200110593f,
     0.0326568342f, 0.0462257591f, 0.0598923724f, 0.0727520563f,
     0.0839025568f, 0.0925291507f, 0.0979839282f, 0.0998501105f,
     0.0979839282f, 0.0925291507f, 0.0839025568f, 0.0727520563f,
     0.0598923724f, 0.0462257591f, 0.0326568342f, 0.0200110593f,
     0.0089656637f, 0.0000000000f,-0.0066302876f,-0.0108939528f,
    -0.0129614398f,-0.0131632271f,-0.0119356508f,-0.0097616817f,
    -0.0071141381f,-0.0044078514f,-0.0019655530f,-0.0000000000f,
     0.0013875383f, 0.0021945208f, 0.0024939045f, 0.0024052586f,
     0.0020652623f, 0.0016020742f, 0.0011169001f, 0.0006745172f,
     0.0003028303f, 0.0000000000f,-0.0002534599f
};

// Passband covers the 0-15 kHz broadcast audio band; attenuation is already
// strong by the 24 kHz output Nyquist frequency.
static const float wbfm_lpf[WBFM_FIR_TAPS] = {
    -1.9782528778e-05f, -2.6904130857e-05f, -2.4046119435e-05f, -4.7693376489e-06f, 3.2926698383e-05f,
    8.3389431244e-05f, 1.3210783336e-04f, 1.5781636390e-04f, 1.3829126686e-04f, 5.8856706311e-05f,
    -7.8787162521e-05f, -2.5071315119e-04f, -4.1112007800e-04f, -5.0081421696e-04f, -4.6335408571e-04f,
    -2.6549568695e-04f, 8.3924233037e-05f, 5.2364245715e-04f, 9.4412370105e-04f, 1.2088619871e-03f,
    1.1904294063e-03f, 8.1338475193e-04f, 9.1949476685e-05f, -8.5088863151e-04f, -1.7934107928e-03f,
    -2.4581376275e-03f, -2.5820746920e-03f, -1.9988629873e-03f, -7.1048711651e-04f, 1.0753445481e-03f,
    2.9606045061e-03f, 4.4315264484e-03f, 4.9824572750e-03f, 4.2626581384e-03f, 2.2080338379e-03f,
    -8.8295157524e-04f, -4.3613453689e-03f, -7.3406764860e-03f, -8.8975686666e-03f, -8.3199778501e-03f,
    -5.3430971019e-03f, -3.0641141581e-04f, 5.8240588527e-03f, 1.1597086657e-02f, 1.5371126500e-02f,
    1.5711226842e-02f, 1.1802764101e-02f, 3.7807127024e-03f, -7.1191654485e-03f, -1.8648779425e-02f,
    -2.7886541853e-02f, -3.1778563994e-02f, -2.7781393541e-02f, -1.4472039939e-02f, 8.0134112932e-03f,
    3.7823623054e-02f, 7.1574054729e-02f, 1.0485168826e-01f, 1.3295539051e-01f, 1.5172984057e-01f,
    1.5832569574e-01f, 1.5172984057e-01f, 1.3295539051e-01f, 1.0485168826e-01f, 7.1574054729e-02f,
    3.7823623054e-02f, 8.0134112932e-03f, -1.4472039939e-02f, -2.7781393541e-02f, -3.1778563994e-02f,
    -2.7886541853e-02f, -1.8648779425e-02f, -7.1191654485e-03f, 3.7807127024e-03f, 1.1802764101e-02f,
    1.5711226842e-02f, 1.5371126500e-02f, 1.1597086657e-02f, 5.8240588527e-03f, -3.0641141581e-04f,
    -5.3430971019e-03f, -8.3199778501e-03f, -8.8975686666e-03f, -7.3406764860e-03f, -4.3613453689e-03f,
    -8.8295157524e-04f, 2.2080338379e-03f, 4.2626581384e-03f, 4.9824572750e-03f, 4.4315264484e-03f,
    2.9606045061e-03f, 1.0753445481e-03f, -7.1048711651e-04f, -1.9988629873e-03f, -2.5820746920e-03f,
    -2.4581376275e-03f, -1.7934107928e-03f, -8.5088863151e-04f, 9.1949476685e-05f, 8.1338475193e-04f,
    1.1904294063e-03f, 1.2088619871e-03f, 9.4412370105e-04f, 5.2364245715e-04f, 8.3924233037e-05f,
    -2.6549568695e-04f, -4.6335408571e-04f, -5.0081421696e-04f, -4.1112007800e-04f, -2.5071315119e-04f,
    -7.8787162521e-05f, 5.8856706311e-05f, 1.3829126686e-04f, 1.5781636390e-04f, 1.3210783336e-04f,
    8.3389431244e-05f, 3.2926698383e-05f, -4.7693376489e-06f, -2.4046119435e-05f, -2.6904130857e-05f,
    -1.9782528778e-05f
};

static int send_rtl_cmd(int s, u8 cmd, u32 param)
{
    u8 pkt[5];
    pkt[0] = cmd;
    u32 be = htonl(param);
    memcpy(pkt + 1, &be, sizeof(be));

    /*
     * rtl_tcp commands are only five bytes, but the socket is nonblocking.
     * A single send() is not guaranteed to write all five bytes and may
     * transiently return EAGAIN/EINPROGRESS.  Losing even one command leaves
     * the server tuned to the old center frequency while the 3DS changes its
     * local DSP state.
     */
    size_t sent = 0;
    u64 deadline = osGetTime() + 150; /* ms */

    while (sent < sizeof(pkt) && aptMainLoop()) {
        int n = send(s, pkt + sent, sizeof(pkt) - sent, 0);

        if (n > 0) {
            sent += (size_t)n;
            continue;
        }

        if (n == 0) {
            g_rtl_cmd_failures++;
            return -1;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK ||
            errno == EINPROGRESS || errno == EALREADY) {
            if (osGetTime() >= deadline) {
                g_rtl_cmd_failures++;
                return -1;
            }
            svcSleepThread(1000000); /* 1 ms */
            continue;
        }

        g_rtl_cmd_failures++;
        return -1;
    }

    if (sent != sizeof(pkt)) g_rtl_cmd_failures++;
    return sent == sizeof(pkt) ? 0 : -1;
}

static bool start_pressed(void)
{
    hidScanInput();
    return (hidKeysDown() & KEY_START) != 0;
}

static int read_exact_cancellable(int s, void *buf, size_t len)
{
    u8 *p = (u8*)buf;
    size_t got = 0;

    while (got < len && aptMainLoop()) {
        if (start_pressed()) return -2;

        int n = recv(s, p + got, len - got, 0);
        if (n > 0) {
            got += (size_t)n;
            continue;
        }
        if (n == 0) return -1;

        if (errno == EAGAIN || errno == EWOULDBLOCK ||
            errno == EINPROGRESS || errno == EALREADY) {
            gspWaitForVBlank();
            continue;
        }
        return -1;
    }

    return got == len ? 0 : -1;
}

static int connect_rtl(const char *host, int port)
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((u16)port);

    if (inet_aton(host, &addr.sin_addr) == 0) {
        printf("Bad IPv4 address: %s\n", host);
        return -1;
    }

    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) {
        printf("socket() failed: %d\n", errno);
        return -1;
    }

    int flags = fcntl(s, F_GETFL, 0);
    if (flags < 0 || fcntl(s, F_SETFL, flags | O_NONBLOCK) < 0) {
        printf("nonblocking setup failed: %d\n", errno);
        close(s);
        return -1;
    }

    printf("Connecting to %s:%d...\n", host, port);
    printf("Press START to exit.\n");

    bool connected = false;
    while (!connected && aptMainLoop()) {
        if (start_pressed()) {
            printf("Connection cancelled.\n");
            close(s);
            return -2;
        }

        int rc = connect(s, (struct sockaddr*)&addr, sizeof(addr));
        if (rc == 0) {
            connected = true;
            break;
        }

        // On a nonblocking socket, these simply mean the connection is still
        // being established. EISCONN means a prior attempt completed.
        if (errno == EISCONN) {
            connected = true;
            break;
        }
        if (errno == EINPROGRESS || errno == EALREADY ||
            errno == EAGAIN || errno == EWOULDBLOCK) {
            gspWaitForVBlank();
            continue;
        }

        printf("connect() failed: %d\n", errno);
        close(s);
        return -1;
    }

    if (!connected) {
        close(s);
        return -1;
    }

    // rtl_tcp sends a 12-byte dongle-info header after connection.
    u8 hdr[12];
    int hr = read_exact_cancellable(s, hdr, sizeof(hdr));
    if (hr == -2) {
        printf("Handshake cancelled.\n");
        close(s);
        return -2;
    }
    if (hr < 0) {
        printf("No rtl_tcp header.\n");
        close(s);
        return -1;
    }

    printf("rtl_tcp magic: %c%c%c%c\n", hdr[0], hdr[1], hdr[2], hdr[3]);
    return s;
}

static inline void iq_fifo_reset(void)
{
    iqFifoRead = 0;
    iqFifoWrite = 0;
    iqFifoCount = 0;
    g_iq_overruns = 0;
    g_iq_latency_drops = 0;
    g_iq_accepted_bytes = 0;
    g_iq_dsp_pairs = 0;
    g_iq_started = false;
    g_iq_credit = 0.0;
    g_iq_pace = 1.0f;
    g_iq_pace_integral = 0.0f;
    g_iq_last_pace_ms = osGetTime();
    g_iq_last_control_ms = g_iq_last_pace_ms;
    g_iq_window_min = IQ_FIFO_BYTES;
    g_iq_window_max = 0;
}

static inline u32 iq_fifo_space(void)
{
    return IQ_FIFO_BYTES - iqFifoCount;
}

static inline void iq_fifo_drop_oldest(u32 count)
{
    if (count > iqFifoCount) count = iqFifoCount;
    /* Always preserve I/Q byte alignment. */
    count &= ~1U;
    if (!count) return;

    iqFifoRead = (iqFifoRead + count) % IQ_FIFO_BYTES;
    iqFifoCount -= count;
    g_iq_latency_drops += count;
}

static u32 iq_fifo_write_bytes_mode(const u8 *src, u32 count, RadioMode mode)
{
    /*
     * NFM is a live scanner path: stale RF is worse than a discontinuity.
     * Enforce a hard 200 ms queue ceiling by dropping the OLDEST IQ before
     * accepting new samples.  WFM retains the original deep elasticity FIFO
     * and never uses this latency trim.
     */
    if (mode == MODE_NFM) {
        count &= ~1U;

        /* If one socket burst itself exceeds the live window, retain only the
         * newest part of that burst. */
        if (count > NFM_IQ_FIFO_MAX_BYTES) {
            u32 skip = count - NFM_IQ_FIFO_MAX_BYTES;
            skip = (skip + 1U) & ~1U;
            src += skip;
            count -= skip;
            g_iq_latency_drops += skip;
        }

        if (iqFifoCount + count > NFM_IQ_FIFO_MAX_BYTES) {
            u32 drop = iqFifoCount + count - NFM_IQ_FIFO_MAX_BYTES;
            drop = (drop + 1U) & ~1U;
            iq_fifo_drop_oldest(drop);
        }
    }

    u32 space = iq_fifo_space();
    if (count > space) {
        g_iq_overruns += count - space;
        count = space;
    }

    u32 first = count;
    u32 tail = IQ_FIFO_BYTES - iqFifoWrite;
    if (first > tail) first = tail;

    if (first)
        memcpy(iqFifo + iqFifoWrite, src, first);

    u32 second = count - first;
    if (second)
        memcpy(iqFifo, src + first, second);

    iqFifoWrite = (iqFifoWrite + count) % IQ_FIFO_BYTES;
    iqFifoCount += count;
    g_iq_accepted_bytes += count;

    if (iqFifoCount < g_iq_window_min) g_iq_window_min = iqFifoCount;
    if (iqFifoCount > g_iq_window_max) g_iq_window_max = iqFifoCount;

    return count;
}

static inline u32 iq_start_bytes_for_mode(RadioMode mode)
{
    return mode == MODE_NFM ? NFM_IQ_FIFO_START_BYTES : IQ_FIFO_START_BYTES;
}

static inline u32 iq_target_bytes_for_mode(RadioMode mode)
{
    return mode == MODE_NFM ? NFM_IQ_FIFO_TARGET_BYTES : IQ_FIFO_TARGET_BYTES;
}

static inline u32 audio_prebuffer_for_mode(RadioMode mode)
{
    return mode == MODE_NFM ? NFM_AUDIO_PREBUFFER_SAMPLES : AUDIO_PREBUFFER_SAMPLES;
}

static inline u32 audio_target_for_mode(RadioMode mode)
{
    return mode == MODE_NFM ? NFM_AUDIO_TARGET_SAMPLES : AUDIO_TARGET_SAMPLES;
}

static inline int ndsp_target_bufs_for_mode(RadioMode mode)
{
    return mode == MODE_NFM ? NFM_NDSP_TARGET_BUFS : NDSP_TARGET_BUFS;
}

static inline void iq_pace_control_update(const Radio *r)
{
    u64 now = osGetTime();
    if (now - g_iq_last_control_ms < IQ_PACE_CONTROL_MS)
        return;

    g_iq_last_control_ms = now;

    const u32 target = iq_target_bytes_for_mode(r->mode);
    float err = ((float)iqFifoCount - (float)target) /
                (float)IQ_FIFO_BYTES;

    g_iq_pace_integral += err;
    if (g_iq_pace_integral > 25.0f) g_iq_pace_integral = 25.0f;
    if (g_iq_pace_integral < -25.0f) g_iq_pace_integral = -25.0f;

    float pace = 1.0f + IQ_PACE_KP * err + IQ_PACE_KI * g_iq_pace_integral;
    if (pace < IQ_PACE_MIN) pace = IQ_PACE_MIN;
    if (pace > IQ_PACE_MAX) pace = IQ_PACE_MAX;

    g_iq_pace += 0.10f * (pace - g_iq_pace);
}

static inline void audio_fifo_reset(void);

static void radio_set_mode(Radio *r, RadioMode mode)
{
    r->mode = mode;
    g_audio_buffer_mode = mode;
    r->prev_i = r->prev_q = 0.0f;
    r->box_i = r->box_q = 0;
    r->box_count = 0;
    r->have_i_byte = false;
    r->audio_count = 0;
    r->audio_accum = 0.0f;
    r->deemph_y = 0.0f;
    memset(r->wbfm_fir, 0, sizeof(r->wbfm_fir));
    r->wbfm_fir_pos = 0;
    r->wbfm_decim_phase = 0;
    memset(r->nfm_i_fir, 0, sizeof(r->nfm_i_fir));
    memset(r->nfm_q_fir, 0, sizeof(r->nfm_q_fir));
    r->nfm_fir_pos = 0;
    r->nfm_decim_phase = 0;
    r->nfm_hp_x1 = 0.0f;
    r->nfm_hp_x2 = 0.0f;
    r->nfm_hp_y1 = 0.0f;
    r->nfm_hp_y2 = 0.0f;
    r->nfm_deemph_y = 0.0f;
    r->nfm_lpf_y = 0.0f;
    memset(r->nfm_i_fir, 0, sizeof(r->nfm_i_fir));
    memset(r->nfm_q_fir, 0, sizeof(r->nfm_q_fir));
    r->nfm_fir_pos = 0;
    r->nfm_decim_phase = 0;
    r->nfm_hp_x1 = 0.0f;
    r->nfm_hp_x2 = 0.0f;
    r->nfm_hp_y1 = 0.0f;
    r->nfm_hp_y2 = 0.0f;
    r->nfm_deemph_y = 0.0f;
    r->nfm_lpf_y = 0.0f;

    // A mode/sample-rate change invalidates buffered PCM from the previous
    // demodulator. Stop NDSP and restart with a clean FIFO.
    ndspChnReset(0);
    ndspChnSetInterp(0, NDSP_INTERP_LINEAR);
    ndspChnSetRate(0, (float)AUDIO_RATE);
    ndspChnSetFormat(0, NDSP_FORMAT_MONO_PCM16);
    memset(waveBuf, 0, sizeof(waveBuf));
    audio_fifo_reset();
    iq_fifo_reset();

    if (mode == MODE_NFM) {
        // 240 ksps complex -> 63-tap channel LPF / 5:1 decimator ->
        // 48 ksps complex -> phase discriminator -> conditioned voice audio.
        r->sample_rate = 240000;
        r->iq_decim = 1;   // NFM now owns its proper FIR decimator below.
        r->audio_decim = 1;

        // Approximately normalize +/-2.5 kHz peak deviation at Fs=48 kHz.
        // This doubles the former +/-5 kHz scaling for narrowband FM.
        r->fm_gain = 2.96f;
    } else {
        // DSP-quality WBFM path:
        // 240 ksps complex IQ -> phase-difference FM discriminator ->
        // 121-tap low-pass / 5:1 decimator -> 48 kHz mono -> de-emphasis.
        //
        // Keeping 240 ksps also holds rtl_tcp traffic near 469 KiB/s.
        r->sample_rate = 240000;
        r->iq_decim = 1;
        r->audio_decim = 5;

        // Convert discriminator radians/sample into approximately normalized
        // audio for ±75 kHz broadcast FM deviation:
        // gain = Fs / (2*pi*75000) = ~0.5093 at 240 ksps.
        r->fm_gain = 0.50929582f;
    }

    // 75 us FM broadcast de-emphasis. In the quality WBFM path it is applied
    // after FIR decimation, so compute the coefficient at 48 kHz.
    if (mode == MODE_WBFM) {
        const float fs = (float)AUDIO_RATE;
        const float dt = 1.0f / fs;
        const float tau = 75e-6f;
        r->deemph_alpha = dt / (tau + dt);
    } else {
        r->deemph_alpha = 1.0f;
    }

    if (r->sock >= 0) {
        send_rtl_cmd(r->sock, 0x02, r->sample_rate); // set sample rate
        send_rtl_cmd(r->sock, 0x01, r->freq_hz);     // set center frequency
        send_rtl_cmd(r->sock, 0x03, 0);              // automatic gain mode
    }
}

static int audio_init(void)
{
    if (ndspInit() != 0) return -1;
    ndspSetOutputMode(NDSP_OUTPUT_MONO);
    ndspChnReset(0);
    ndspChnSetInterp(0, NDSP_INTERP_LINEAR);
    ndspChnSetRate(0, (float)AUDIO_RATE);
    ndspChnSetFormat(0, NDSP_FORMAT_MONO_PCM16);

    audioMem = (s16*)linearAlloc(AUDIO_BUFS * AUDIO_SAMPLES * sizeof(s16));
    if (!audioMem) return -1;
    memset(waveBuf, 0, sizeof(waveBuf));
    memset(audioMem, 0, AUDIO_BUFS * AUDIO_SAMPLES * sizeof(s16));
    audio_fifo_reset();
    return 0;
}

static inline bool wavebuf_busy(const ndspWaveBuf *wb)
{
    return wb->status == NDSP_WBUF_QUEUED ||
           wb->status == NDSP_WBUF_PLAYING;
}

static inline void audio_fifo_reset(void)
{
    audioFifoRead = 0;
    audioFifoWrite = 0;
    audioFifoCount = 0;
    g_audio_overruns = 0;
    g_audio_rebuffers = 0;
    g_audio_starved_ms = 0;
    g_starve_start_ms = 0;
    g_audio_started_once = false;
    g_audio_rebuffering = true;

    g_resamp_step = 1.0f;
    g_resamp_integral = 0.0f;
    g_resamp_phase = 0.0f;
    g_resamp_prev = 0.0f;
    g_resamp_have_prev = false;
    g_resamp_last_control_ms = osGetTime();
    g_pcm_source_samples = 0;
    g_pcm_output_samples = 0;

    g_fifo_window_min = AUDIO_FIFO_SAMPLES;
    g_fifo_window_max = 0;
    audioWrite = 0;
}

static inline void audio_fifo_push_s16(s16 sample)
{
    if (audioFifoCount >= AUDIO_FIFO_SAMPLES) {
        // This is a true producer-overrun: DSP is generating PCM faster than
        // NDSP can consume it for longer than the FIFO can absorb.
        g_audio_overruns++;
        return;
    }

    audioFifo[audioFifoWrite] = sample;
    audioFifoWrite++;
    if (audioFifoWrite >= AUDIO_FIFO_SAMPLES)
        audioFifoWrite = 0;
    audioFifoCount++;

    if (audioFifoCount < g_fifo_window_min)
        g_fifo_window_min = audioFifoCount;
    if (audioFifoCount > g_fifo_window_max)
        g_fifo_window_max = audioFifoCount;
}

static inline void resampler_control_update(void)
{
    u64 now = osGetTime();
    if (now - g_resamp_last_control_ms < RESAMP_CONTROL_MS)
        return;

    g_resamp_last_control_ms = now;

    // Positive error = FIFO too full -> step grows -> slightly fewer output
    // samples. Negative error = FIFO too empty -> step shrinks -> slightly
    // more output samples.
    const u32 target = audio_target_for_mode(g_audio_buffer_mode);
    float err = ((float)audioFifoCount - (float)target) /
                (float)AUDIO_FIFO_SAMPLES;

    // PI control handles both short occupancy displacement and persistent
    // oscillator mismatch. Integral is deliberately clamped and slow.
    g_resamp_integral += err;
    if (g_resamp_integral > 20.0f) g_resamp_integral = 20.0f;
    if (g_resamp_integral < -20.0f) g_resamp_integral = -20.0f;

    float step = 1.0f + RESAMP_KP * err + RESAMP_KI * g_resamp_integral;
    if (step < RESAMP_MIN_STEP) step = RESAMP_MIN_STEP;
    if (step > RESAMP_MAX_STEP) step = RESAMP_MAX_STEP;

    // Smooth control changes so the pitch cannot jump from FIFO jitter.
    g_resamp_step += 0.10f * (step - g_resamp_step);
}

static inline void push_audio(float x)
{
    if (!g_audio_gate_open) return;
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;

    g_pcm_source_samples++;
    resampler_control_update();

    if (!g_resamp_have_prev) {
        g_resamp_prev = x;
        g_resamp_have_prev = true;
        return;
    }

    // Treat successive demodulated 48 kHz samples as endpoints of one source
    // interval. Emit at fractional positions; step > 1 emits microscopically
    // fewer samples, step < 1 emits microscopically more.
    while (g_resamp_phase < 1.0f) {
        float y = g_resamp_prev + (x - g_resamp_prev) * g_resamp_phase;
        if (y > 1.0f) y = 1.0f;
        if (y < -1.0f) y = -1.0f;
        audio_fifo_push_s16((s16)(y * 26000.0f));
        g_pcm_output_samples++;
        g_resamp_phase += g_resamp_step;
    }

    g_resamp_phase -= 1.0f;
    g_resamp_prev = x;
}

static int count_busy_wavebufs(void)
{
    int busy = 0;
    for (int n = 0; n < AUDIO_BUFS; n++) {
        if (wavebuf_busy(&waveBuf[n]))
            busy++;
    }
    return busy;
}

static void begin_rebuffer(void)
{
    if (g_audio_rebuffering)
        return;

    g_audio_rebuffering = true;
    g_audio_rebuffers++;
    g_starve_start_ms = osGetTime();
}

static void finish_rebuffer(void)
{
    if (!g_audio_rebuffering)
        return;

    // Startup prebuffer is intentional latency, not starvation. Only add time
    // after audio has previously started and then genuinely run dry.
    if (g_audio_started_once && g_starve_start_ms != 0) {
        u64 now = osGetTime();
        if (now >= g_starve_start_ms)
            g_audio_starved_ms += now - g_starve_start_ms;
    }

    g_starve_start_ms = 0;
    g_audio_rebuffering = false;
    g_audio_started_once = true;
}

static u64 current_starved_ms(void)
{
    u64 total = g_audio_starved_ms;
    if (g_audio_rebuffering && g_audio_started_once && g_starve_start_ms != 0) {
        u64 now = osGetTime();
        if (now >= g_starve_start_ms)
            total += now - g_starve_start_ms;
    }
    return total;
}

static void audio_pump(void)
{
    int busy = count_busy_wavebufs();

    // WFM retains the public beta's deep startup/recovery reservoir. NFM uses
    // a much smaller reservoir so voice/scanner reception feels responsive.
    const u32 prebuffer = audio_prebuffer_for_mode(g_audio_buffer_mode);
    const int target_bufs = ndsp_target_bufs_for_mode(g_audio_buffer_mode);
    if (g_audio_rebuffering) {
        if (audioFifoCount < prebuffer)
            return;

        finish_rebuffer();
        busy = count_busy_wavebufs();
    }

    // WFM keeps the original ~85 ms NDSP queue; NFM keeps about ~43 ms.
    // The remainder stays in the software FIFO as the jitter reservoir.
    while (audioFifoCount >= AUDIO_SAMPLES && busy < target_bufs) {
        int chosen = -1;

        for (int n = 0; n < AUDIO_BUFS; n++) {
            int idx = (audioWrite + n) % AUDIO_BUFS;
            if (!wavebuf_busy(&waveBuf[idx])) {
                chosen = idx;
                break;
            }
        }

        if (chosen < 0)
            break;

        s16 *dst = audioMem + chosen * AUDIO_SAMPLES;
        for (int n = 0; n < AUDIO_SAMPLES; n++) {
            dst[n] = audioFifo[audioFifoRead];
            audioFifoRead++;
            if (audioFifoRead >= AUDIO_FIFO_SAMPLES)
                audioFifoRead = 0;
        }
        audioFifoCount -= AUDIO_SAMPLES;
        if (audioFifoCount < g_fifo_window_min)
            g_fifo_window_min = audioFifoCount;

        DSP_FlushDataCache(dst, AUDIO_SAMPLES * sizeof(s16));

        memset(&waveBuf[chosen], 0, sizeof(ndspWaveBuf));
        waveBuf[chosen].data_vaddr = dst;
        waveBuf[chosen].nsamples = AUDIO_SAMPLES;
        ndspChnWaveBufAdd(0, &waveBuf[chosen]);

        audioWrite = (chosen + 1) % AUDIO_BUFS;
        busy++;
    }

    // A genuine underrun is when NDSP has consumed every queued block and the
    // FIFO cannot provide another complete one. Re-enter the mode-specific
    // rebuffer threshold rather than restarting on one tiny chunk.
    if (g_audio_started_once && busy == 0 && audioFifoCount < AUDIO_SAMPLES) {
        begin_rebuffer();
    }
}

static inline float fast_atan2f_quality(float y, float x)
{
    // atan approximation on [0,1], then quadrant reconstruction.
    // Much closer to the true phase difference than the old derivative
    // discriminator, while avoiding a libm atan2f() call for every IQ sample.
    const float PI_2 = 1.5707963267948966f;
    const float PI   = 3.1415926535897932f;

    float ax = fabsf(x);
    float ay = fabsf(y);
    if (ax < 1.0e-20f && ay < 1.0e-20f)
        return 0.0f;

    bool y_bigger = ay > ax;
    float mx = y_bigger ? ay : ax;
    float mn = y_bigger ? ax : ay;
    float a = mn / mx;
    float s = a * a;

    // Minimax-style polynomial approximation of atan(a) over [0,1].
    float ang = (((-0.0464964749f * s + 0.15931422f) * s
                  - 0.327622764f) * s * a) + a;

    if (y_bigger)
        ang = PI_2 - ang;
    if (x < 0.0f)
        ang = PI - ang;
    if (y < 0.0f)
        ang = -ang;

    return ang;
}

static inline float fm_discriminator_quality(Radio *r, float i, float q)
{
    // Phase of current_sample * conj(previous_sample):
    //   real = I*prevI + Q*prevQ
    //   imag = Q*prevI - I*prevQ
    //
    // atan2(imag, real) gives the true wrapped phase increment in [-pi, pi].
    // This avoids the severe sin(delta)-like nonlinearity of the old
    // derivative approximation at WBFM's large phase excursions.
    float real = i * r->prev_i + q * r->prev_q;
    float imag = q * r->prev_i - i * r->prev_q;

    r->prev_i = i;
    r->prev_q = q;

    if (real == 0.0f && imag == 0.0f)
        return 0.0f;

    return fast_atan2f_quality(imag, real) * r->fm_gain;
}

static inline float wbfm_fir_decimate_5(Radio *r, float sample, bool *have_output)
{
    r->wbfm_fir[r->wbfm_fir_pos] = sample;
    r->wbfm_fir_pos++;
    if (r->wbfm_fir_pos >= WBFM_FIR_TAPS)
        r->wbfm_fir_pos = 0;

    r->wbfm_decim_phase++;
    if (r->wbfm_decim_phase < 5) {
        *have_output = false;
        return 0.0f;
    }
    r->wbfm_decim_phase = 0;

    // Only evaluate the FIR for samples we actually keep: 48,000 outputs/s
    // instead of 240,000 full convolutions/s.
    float acc = 0.0f;
    int idx = r->wbfm_fir_pos - 1;
    if (idx < 0) idx = WBFM_FIR_TAPS - 1;

    for (int k = 0; k < WBFM_FIR_TAPS; k++) {
        acc += wbfm_lpf[k] * r->wbfm_fir[idx];
        if (--idx < 0) idx = WBFM_FIR_TAPS - 1;
    }

    *have_output = true;
    return acc;
}

static inline bool nfm_iq_fir_decimate_5(Radio *r, float i, float q,
                                           float *oi, float *oq)
{
    r->nfm_i_fir[r->nfm_fir_pos] = i;
    r->nfm_q_fir[r->nfm_fir_pos] = q;
    r->nfm_fir_pos++;
    if (r->nfm_fir_pos >= NFM_IQ_FIR_TAPS)
        r->nfm_fir_pos = 0;

    if (++r->nfm_decim_phase < 5)
        return false;
    r->nfm_decim_phase = 0;

    float ai = 0.0f, aq = 0.0f;
    int idx = r->nfm_fir_pos - 1;
    if (idx < 0) idx = NFM_IQ_FIR_TAPS - 1;
    for (int k = 0; k < NFM_IQ_FIR_TAPS; k++) {
        float h = nfm_iq_lpf[k];
        ai += h * r->nfm_i_fir[idx];
        aq += h * r->nfm_q_fir[idx];
        if (--idx < 0) idx = NFM_IQ_FIR_TAPS - 1;
    }

    *oi = ai;
    *oq = aq;
    return true;
}

static inline float nfm_voice_process(Radio *r, float x)
{
    /*
     * Communications FM transmitters pre-emphasize speech and discriminator
     * audio contains plenty of low-frequency wander plus high-frequency hiss.
     * Shape it into scanner-like voice audio before it reaches NDSP.
     */

    // ~300 Hz 2nd-order Butterworth high-pass at 48 kHz.  Compared with
    // the former one-pole 250 Hz stage this rejects discriminator/DC rumble
    // and sub-audible signalling much more strongly while keeping normal
    // communications speech essentially flat above the cutoff region.
    const float hp_b0 =  0.97261390f;
    const float hp_b1 = -1.94522780f;
    const float hp_b2 =  0.97261390f;
    const float hp_a1 = -1.94447766f;
    const float hp_a2 =  0.94597794f;
    float hp = hp_b0*x + hp_b1*r->nfm_hp_x1 + hp_b2*r->nfm_hp_x2
             - hp_a1*r->nfm_hp_y1 - hp_a2*r->nfm_hp_y2;
    r->nfm_hp_x2 = r->nfm_hp_x1;
    r->nfm_hp_x1 = x;
    r->nfm_hp_y2 = r->nfm_hp_y1;
    r->nfm_hp_y1 = hp;

    // 75 us de-emphasis at 48 kHz.  This takes the harsh edge/hiss off normal
    // pre-emphasized FM voice without the very heavy muffling of a long RC.
    const float deemph_a = 0.2170f;
    r->nfm_deemph_y += deemph_a * (hp - r->nfm_deemph_y);

    // Additional ~4.2 kHz low-pass to suppress out-of-band discriminator hiss.
    // One pole is deliberately modest; de-emphasis is already doing most of
    // the high-frequency shaping.
    const float lp_a = 0.3546f;
    r->nfm_lpf_y += lp_a * (r->nfm_deemph_y - r->nfm_lpf_y);

    // Narrow-FM voice still has useful headroom after de-emphasis/filtering.
    // Add ~+4.1 dB of explicit audio makeup before the existing soft limiter.
    // This raises speech level without changing discriminator/deviation math.
    float y = r->nfm_lpf_y * 1.60f;

    // Mild soft limiting: catches occasional discriminator spikes without the
    // hard crackle produced by clipping directly at +/-1.
    float ay = fabsf(y);
    if (ay > 0.82f) {
        float over = ay - 0.82f;
        ay = 0.82f + over / (1.0f + 3.0f * over);
        y = y < 0.0f ? -ay : ay;
    }
    return y;
}

static inline void fft_capture_sample(float i, float q);

static inline void process_complex_sample(Radio *r, float i, float q)
{
    if (r->mode == MODE_NFM) {
        /*
         * Keep squelch measurement ahead of the NFM channel FIR so its
         * adjacent-noise reference is not erased by the very filter whose
         * output is being judged.  The NFM demod/audio DSP below is unchanged.
         */
        scanner_channel_metric_sample(r->mode, i, q);

        float fi, fq;
        if (!nfm_iq_fir_decimate_5(r, i, q, &fi, &fq))
            return;

        // Everything below now sees the selected ~12 kHz NFM channel at 48 ksps.
        if (!fftReady)
            fft_capture_sample(fi, fq);

        float a = fm_discriminator_quality(r, fi, fq);
        push_audio(nfm_voice_process(r, a));
        return;
    }

    scanner_channel_metric_sample(r->mode,i,q);

    // WFM scope follows the paced 240 ksps IQ stream.
    if (!fftReady)
        fft_capture_sample(i, q);

    float a = fm_discriminator_quality(r, i, q);

    bool have_output;
    float out = wbfm_fir_decimate_5(r, a, &have_output);
    if (!have_output)
        return;

    // Standard US broadcast-FM 75 us de-emphasis at the 48 kHz audio rate.
    r->deemph_y += r->deemph_alpha * (out - r->deemph_y);
    push_audio(r->deemph_y);
}

static inline void process_iq_pair(Radio *r, u8 ib, u8 qb)
{
    // Both modes intentionally run from the same 240 ksps raw complex stream.
    // WFM processes every sample directly; NFM performs a proper 63-tap 5:1
    // complex FIR decimation inside process_complex_sample().
    float i = ((float)ib - 128.0f) * (1.0f / 128.0f);
    float q = ((float)qb - 128.0f) * (1.0f / 128.0f);
    process_complex_sample(r, i, q);
}

static u32 process_iq(Radio *r, const u8 *buf, int bytes)
{
    int n = 0;
    u32 pairs = 0;

    // TCP is a byte stream: recv() is allowed to return an odd number of
    // bytes. Preserve I/Q alignment across calls instead of assuming each
    // recv begins on an I byte.
    if (r->have_i_byte && bytes > 0) {
        process_iq_pair(r, r->i_byte, buf[0]);
        r->have_i_byte = false;
        n = 1;
        pairs++;
    }

    for (; n + 1 < bytes; n += 2) {
        process_iq_pair(r, buf[n], buf[n + 1]);
        pairs++;
    }

    if (n < bytes) {
        r->i_byte = buf[n];
        r->have_i_byte = true;
    }

    return pairs;
}

static u32 iq_process_paced(Radio *r, u8 *scratch, u32 scratchBytes)
{
    if (!g_iq_started) {
        if (iqFifoCount < iq_start_bytes_for_mode(r->mode))
            return 0;

        g_iq_started = true;
        g_iq_last_pace_ms = osGetTime();
        g_iq_credit = 0.0;
    }

    iq_pace_control_update(r);

    u64 now = osGetTime();
    u64 elapsed = now - g_iq_last_pace_ms;

    if (elapsed > 0) {
        g_iq_credit += ((double)r->sample_rate * (double)g_iq_pace *
                        (double)elapsed) / 1000.0;

        // Never allow more than 100 ms of catch-up credit at once.
        double cap = (double)r->sample_rate * 0.10;
        if (g_iq_credit > cap)
            g_iq_credit = cap;

        g_iq_last_pace_ms = now;
    }

    u32 availablePairs = iqFifoCount / 2U;
    u32 duePairs = (u32)g_iq_credit;
    if (duePairs > availablePairs) duePairs = availablePairs;

    u32 maxPairs = scratchBytes / 2U;
    if (duePairs > maxPairs) duePairs = maxPairs;
    if (duePairs == 0)
        return 0;

    u32 bytes = duePairs * 2U;

    u32 first = bytes;
    u32 tail = IQ_FIFO_BYTES - iqFifoRead;
    if (first > tail) first = tail;

    memcpy(scratch, iqFifo + iqFifoRead, first);

    u32 second = bytes - first;
    if (second)
        memcpy(scratch + first, iqFifo, second);

    iqFifoRead = (iqFifoRead + bytes) % IQ_FIFO_BYTES;
    iqFifoCount -= bytes;

    if (iqFifoCount < g_iq_window_min)
        g_iq_window_min = iqFifoCount;

    u32 processed = process_iq(r, scratch, (int)bytes);
    g_iq_dsp_pairs += processed;

    g_iq_credit -= (double)processed;
    if (g_iq_credit < 0.0)
        g_iq_credit = 0.0;

    return processed;
}


static inline void fft_capture_sample(float i, float q)
{
    if (fftFill >= FFT_N)
        return;

    // Hann window while capturing.
    float w = 0.5f - 0.5f * cosf((2.0f * 3.14159265358979323846f *
                                  (float)fftFill) / (float)(FFT_N - 1));
    fftRe[fftFill] = i * w;
    fftIm[fftFill] = q * w;
    fftFill++;

    if (fftFill >= FFT_N)
        fftReady = true;
}

static void fft_compute(void)
{
    // In-place radix-2 Cooley-Tukey FFT.
    for (u32 i = 1, j = 0; i < FFT_N; i++) {
        u32 bit = FFT_N >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;

        if (i < j) {
            float tr = fftRe[i]; fftRe[i] = fftRe[j]; fftRe[j] = tr;
            float ti = fftIm[i]; fftIm[i] = fftIm[j]; fftIm[j] = ti;
        }
    }

    for (u32 len = 2; len <= FFT_N; len <<= 1) {
        float ang = -2.0f * 3.14159265358979323846f / (float)len;
        float wlenR = cosf(ang);
        float wlenI = sinf(ang);

        for (u32 i = 0; i < FFT_N; i += len) {
            float wr = 1.0f;
            float wi = 0.0f;

            for (u32 j = 0; j < len / 2; j++) {
                u32 a = i + j;
                u32 b = a + len / 2;

                float vr = fftRe[b] * wr - fftIm[b] * wi;
                float vi = fftRe[b] * wi + fftIm[b] * wr;
                float ur = fftRe[a];
                float ui = fftIm[a];

                fftRe[a] = ur + vr;
                fftIm[a] = ui + vi;
                fftRe[b] = ur - vr;
                fftIm[b] = ui - vi;

                float nwr = wr * wlenR - wi * wlenI;
                wi = wr * wlenI + wi * wlenR;
                wr = nwr;
            }
        }
    }

    // FFT shift + log-ish magnitude.
    for (u32 x = 0; x < FFT_N; x++) {
        u32 k = (x + FFT_N / 2) & (FFT_N - 1);
        float p = fftRe[k] * fftRe[k] + fftIm[k] * fftIm[k];
        float db = 10.0f * log10f(p + 1.0e-12f);
        fftMag[x] = db;
    }
}

// ---------------------------------------------------------------------------
// Unified framebuffer UI (v21.8a)
// One renderer owns both LCDs. This removes mixed text/raw-framebuffer
// ownership switching, which was the main source of the recurring artifacts.
// ---------------------------------------------------------------------------

typedef struct { u8 r,g,b; } UiColor;
typedef struct { u8 *fb; int w,h; } UiSurface;
typedef struct { char ch; u8 row[7]; } UiGlyph;

static const UiColor C_BG={0,0,0}, C_PANEL={12,18,26}, C_TEXT={224,232,240},
    C_DIM={125,145,162}, C_ACCENT={75,205,240}, C_WARN={245,194,80},
    C_OK={90,220,125}, C_SELECT={42,73,96}, C_GRID={38,48,58},
    C_ORANGE={245,145,55}, C_RED={240,72,72};

static const UiGlyph g_font5x7[] = {
    {' ', {0,0,0,0,0,0,0}},
    {'!', {4,4,4,4,4,0,4}},
    {'#', {10,31,10,10,31,10,10}},
    {'%', {25,26,4,8,22,6,0}},
    {'&', {12,18,20,8,21,18,13}},
    {'\'', {4,4,0,0,0,0,0}},
    {'(', {2,4,8,8,8,4,2}},
    {')', {8,4,2,2,2,4,8}},
    {'+', {0,4,4,31,4,4,0}},
    {',', {0,0,0,0,6,4,8}},
    {'-', {0,0,0,31,0,0,0}},
    {'.', {0,0,0,0,0,6,6}},
    {'/', {1,2,4,8,16,0,0}},
    {':', {0,6,6,0,6,6,0}},
    {';', {0,6,6,0,6,4,8}},
    {'<', {2,4,8,16,8,4,2}},
    {'=', {0,31,0,31,0,0,0}},
    {'>', {8,4,2,1,2,4,8}},
    {'?', {14,17,1,2,4,0,4}},
    {'[', {14,8,8,8,8,8,14}},
    {']', {14,2,2,2,2,2,14}},
    {'_', {0,0,0,0,0,0,31}},
    {'|', {4,4,4,4,4,4,4}},
    {'0', {14,17,19,21,25,17,14}},
    {'1', {4,12,4,4,4,4,14}},
    {'2', {14,17,1,2,4,8,31}},
    {'3', {30,1,1,14,1,1,30}},
    {'4', {2,6,10,18,31,2,2}},
    {'5', {31,16,16,30,1,1,30}},
    {'6', {14,16,16,30,17,17,14}},
    {'7', {31,1,2,4,8,8,8}},
    {'8', {14,17,17,14,17,17,14}},
    {'9', {14,17,17,15,1,1,14}},
    {'A', {14,17,17,31,17,17,17}},
    {'B', {30,17,17,30,17,17,30}},
    {'C', {14,17,16,16,16,17,14}},
    {'D', {30,17,17,17,17,17,30}},
    {'E', {31,16,16,30,16,16,31}},
    {'F', {31,16,16,30,16,16,16}},
    {'G', {14,17,16,23,17,17,14}},
    {'H', {17,17,17,31,17,17,17}},
    {'I', {14,4,4,4,4,4,14}},
    {'J', {7,2,2,2,2,18,12}},
    {'K', {17,18,20,24,20,18,17}},
    {'L', {16,16,16,16,16,16,31}},
    {'M', {17,27,21,21,17,17,17}},
    {'N', {17,25,21,19,17,17,17}},
    {'O', {14,17,17,17,17,17,14}},
    {'P', {30,17,17,30,16,16,16}},
    {'Q', {14,17,17,17,21,18,13}},
    {'R', {30,17,17,30,20,18,17}},
    {'S', {15,16,16,14,1,1,30}},
    {'T', {31,4,4,4,4,4,4}},
    {'U', {17,17,17,17,17,17,14}},
    {'V', {17,17,17,17,17,10,4}},
    {'W', {17,17,17,21,21,21,10}},
    {'X', {17,17,10,4,10,17,17}},
    {'Y', {17,17,10,4,4,4,4}},
    {'Z', {31,1,2,4,8,16,31}}
};
#define FONT_COUNT ((int)(sizeof(g_font5x7)/sizeof(g_font5x7[0])))

typedef struct {
    u32 sock1,sock10,accept1,accept10,dsp1,dsp10;
    u32 iqPct,iqMinPct,iqMaxPct,afPct,afMinPct,afMaxPct;
    u32 cpuPct,memUsedMb,memFreeMb;
    long ppm; u64 droppedBytes; double dropPct; u32 pcmSrc,pcmOut;
} DiagnosticSnapshot;
static DiagnosticSnapshot g_diag;

static UiSurface ui_surface(gfxScreen_t screen)
{
    u16 rw=0,rh=0;
    UiSurface s;
    s.fb=gfxGetFramebuffer(screen,GFX_LEFT,&rw,&rh);
    s.w=(screen==GFX_TOP)?400:320; s.h=240;
    (void)rw; (void)rh;
    return s;
}

static inline void ui_pixel(UiSurface *s,int x,int y,UiColor c)
{
    if(!s->fb||x<0||x>=s->w||y<0||y>=s->h)return;
    u32 pixel=(u32)(x*240+(239-y));
    u32 pos=pixel*3U;
    s->fb[pos+0]=c.b; s->fb[pos+1]=c.g; s->fb[pos+2]=c.r;
}

static void ui_clear(UiSurface *s,UiColor c)
{
    if(!s->fb)return;
    if(c.r==0&&c.g==0&&c.b==0) memset(s->fb,0,(size_t)s->w*240U*3U);
    else for(int x=0;x<s->w;x++)for(int y=0;y<s->h;y++)ui_pixel(s,x,y,c);
}

static void ui_rect(UiSurface *s,int x,int y,int w,int h,UiColor c)
{
    int x1=x+w,y1=y+h; if(x<0)x=0;if(y<0)y=0;if(x1>s->w)x1=s->w;if(y1>s->h)y1=s->h;
    for(int xx=x;xx<x1;xx++)for(int yy=y;yy<y1;yy++)ui_pixel(s,xx,yy,c);
}

static void ui_line(UiSurface *s,int x0,int y0,int x1,int y1,UiColor c)
{
    int dx=abs(x1-x0), sx=x0<x1?1:-1;
    int dy=-abs(y1-y0), sy=y0<y1?1:-1; int err=dx+dy;
    for(;;){ ui_pixel(s,x0,y0,c); if(x0==x1&&y0==y1)break; int e2=2*err;
        if(e2>=dy){err+=dy;x0+=sx;} if(e2<=dx){err+=dx;y0+=sy;} }
}

static const u8 *ui_glyph(char ch)
{
    static const u8 unknown[7]={14,17,1,2,4,0,4};
    if(ch>='a'&&ch<='z')ch=(char)(ch-'a'+'A');
    for(int i=0;i<FONT_COUNT;i++)if(g_font5x7[i].ch==ch)return g_font5x7[i].row;
    return unknown;
}

static int ui_text_width(const char *s,int scale)
{
    int n=(int)strlen(s); return n? n*(6*scale)-scale:0;
}

static void ui_text(UiSurface *s,int x,int y,const char *text,int scale,UiColor c)
{
    for(;*text;text++){
        const u8 *g=ui_glyph(*text);
        for(int ry=0;ry<7;ry++)for(int rx=0;rx<5;rx++)if(g[ry]&(1<<(4-rx)))
            ui_rect(s,x+rx*scale,y+ry*scale,scale,scale,c);
        x+=6*scale; if(x>=s->w)break;
    }
}

static void ui_textf(UiSurface *s,int x,int y,int scale,UiColor c,const char *fmt,...)
{
    char buf[128]; va_list ap; va_start(ap,fmt); vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap);
    ui_text(s,x,y,buf,scale,c);
}

static void ui_center(UiSurface *s,int y,const char *text,int scale,UiColor c)
{ int x=(s->w-ui_text_width(text,scale))/2; if(x<2)x=2; ui_text(s,x,y,text,scale,c); }

static void ui_frequency_hz(UiSurface *s,u32 hz,int y)
{
    char digits[24]; snprintf(digits,sizeof(digits),"%03lu.%06lu",
        (unsigned long)(hz/1000000U),(unsigned long)(hz%1000000U));
    const int ds=4, us=2, gap=9;
    int dw=ui_text_width(digits,ds), uw=ui_text_width("MHz",us);
    int x=(s->w-(dw+gap+uw))/2; if(x<2)x=2;
    ui_text(s,x,y,digits,ds,C_ACCENT);
    ui_text(s,x+dw+gap,y+12,"MHz",us,C_ACCENT);
}

static void ui_frequency(UiSurface *s,const Radio *r,int y)
{ ui_frequency_hz(s,r->freq_hz,y); }

static void battery_poll(void)
{
    if(!g_ptmu_ready) return;
    u64 now=osGetTime();
    if(g_battery_last_poll_ms && now-g_battery_last_poll_ms<2000) return;

    u8 level=g_battery_level;
    u8 charging=g_battery_charging;
    if(R_SUCCEEDED(PTMU_GetBatteryLevel(&level))) g_battery_level=level;
    if(R_SUCCEEDED(PTMU_GetBatteryChargeState(&charging))) g_battery_charging=charging;
    g_battery_last_poll_ms=now;
}

static void ui_play_stop_icon(UiSurface *s)
{
    /* Transport icon shows the CURRENT receiver state:
     * running => green PLAY triangle, stopped => red STOP square. */
    if(g_receiver_running){
        UiColor c=C_OK;
        /* Right-pointing play triangle: flat edge left, tip right. */
        for(int dx=0;dx<12;dx++){
            int half=(11-dx)/2;
            int y0=35-half;
            int y1=35+half;
            ui_line(s,11+dx,y0,11+dx,y1,c);
        }
    }else{
        ui_rect(s,12,29,12,12,C_RED);
    }
}

static void ui_battery_icon(UiSurface *s)
{
    battery_poll();

    const int x=s->w-31, y=28, w=23, h=14;
    UiColor c=C_DIM;
    if(g_ptmu_ready){
        if(g_battery_charging) c=C_ORANGE;
        else if(g_battery_level<=1) c=C_RED;
        else if(g_battery_level<=3) c=C_WARN;
        else c=C_OK;
    }

    /* Outline + terminal. */
    ui_line(s,x,y,x+w-1,y,c);
    ui_line(s,x,y+h-1,x+w-1,y+h-1,c);
    ui_line(s,x,y,x,y+h-1,c);
    ui_line(s,x+w-1,y,x+w-1,y+h-1,c);
    ui_rect(s,x+w,y+4,3,6,c);

    if(!g_ptmu_ready) return;

    /* PTMU battery level is coarse (0..5). Draw that honestly as five bars. */
    int bars=(int)g_battery_level;
    if(bars<0) bars=0;
    if(bars>5) bars=5;
    for(int i=0;i<bars;i++) ui_rect(s,x+3+i*4,y+3,3,h-6,c);
}

static void ui_radio_status(UiSurface *s,const Radio *r,u32 iqPct,u32 afPct)
{
    ui_clear(s,C_BG);

    const char *state;
    UiColor sc;
    u64 bannerNow=osGetTime();
    if(!g_receiver_running){
        state="STOPPED";
        sc=C_RED;
    }else if(g_receiver_resuming_until_ms && bannerNow<g_receiver_resuming_until_ms){
        state="RESUMING";
        sc=C_WARN;
    }else if(g_scan.vfo_mode){
        state=g_scan.squelch_open?"VFO RX":"VFO HOLD";
        sc=g_scan.squelch_open?C_OK:C_TEXT;
    }else if(scanner_is_hold(&g_scan)){
        state=g_scan.squelch_open?"HOLD RX":"HOLD";
        sc=g_scan.squelch_open?C_OK:C_TEXT;
    }else{
        state=g_scan.squelch_open?"RECEIVE":"SCANNING";
        sc=g_scan.squelch_open?C_OK:C_WARN;
    }

    /* Status banner is now the primary top-screen information. */
    ui_line(s,34,18,s->w-34,18,C_PANEL);
    ui_play_stop_icon(s);
    ui_center(s,24,state,3,sc);
    ui_battery_icon(s);
    ui_line(s,34,51,s->w-34,51,C_PANEL);

    if(g_scan.vfo_mode){
        ui_text(s,8,66,"VFO",2,C_TEXT);
        ui_textf(s,8,86,2,C_DIM,"STEP %.3f kHz",
                 (double)g_step_hz[g_scan.vfo_step_index]/1000.0);
        ui_textf(s,8,111,2,C_ACCENT,"%03lu.%06lu MHz  %s",
                 (unsigned long)(r->freq_hz/1000000U),
                 (unsigned long)(r->freq_hz%1000000U),
                 r->mode==MODE_NFM?"NFM":"WFM");
    } else {
        ScanBank *b=scanner_bank(&g_scan);
        ScanChannel *c=scanner_channel(&g_scan);

        ui_textf(s,8,65,2,C_DIM,"BANK %d/%d",
                 g_scan.bank_index+1,g_scan.bank_count);
        if(b) ui_text(s,150,65,b->name,2,C_TEXT);

        if(c){
            ui_textf(s,8,86,2,C_DIM,"CH %d/%d",
                     g_scan.channel_index+1,b->channel_count);
            ui_text(s,150,86,c->name,2,C_TEXT);

            /* Frequency remains visible on top, but no longer dominates it. */
            ui_textf(s,8,111,2,C_ACCENT,"%03lu.%06lu MHz  %s",
                     (unsigned long)(c->freq_hz/1000000U),
                     (unsigned long)(c->freq_hz%1000000U),
                     c->mode==MODE_NFM?"NFM":"WFM");
        }else{
            ui_text(s,8,111,"NO CHANNEL",2,C_WARN);
        }
    }

    {
        bool sqlOvr=false;
        int effSql=scanner_effective_sql(&g_scan,&sqlOvr);
        ui_textf(s,8,139,2,g_scan.squelch_open?C_OK:C_DIM,
                 "SQ %s   SQL %d%s",
                 g_scan.squelch_open?"OPEN":"CLOSED",
                 effSql,sqlOvr?" OVR":"");
    }

    ui_textf(s,8,160,2,C_TEXT,"RF %.1f  NF %.1f",
             (double)g_scan.rf_db,(double)g_scan.noise_db);

    ui_textf(s,8,181,2,C_TEXT,"M %+.1f dB  %s",
             (double)g_scan.margin_db,
             scan_resume_name(g_scan.resume_mode));

    if(g_scan.squelch_open && !g_scan.vfo_mode){
        ScanChannel *c=scanner_channel(&g_scan);
        if(c) ui_textf(s,8,203,1,C_OK,"HIT #%lu",
                       (unsigned long)c->hit_count);
    }

    {
        ScanBank *b=scanner_bank(&g_scan);
        ui_textf(s,8,216,1,C_DIM,"ELIG %d / TOTAL %d  BANK ELIG %d / %d",
                 scanner_eligible_count(&g_scan),
                 scanner_total_channel_count(&g_scan),
                 scanner_bank_eligible_count(&g_scan,g_scan.bank_index),
                 b?b->channel_count:0);
    }

    ui_textf(s,8,229,1,C_DIM,"IQ %lu%%  AUDIO %lu%%  LOAD %lu%%  MEM U%luM F%luM",
             (unsigned long)iqPct,(unsigned long)afPct,
             (unsigned long)g_diag.cpuPct,
             (unsigned long)g_diag.memUsedMb,(unsigned long)g_diag.memFreeMb);
}

static void ui_control_status(UiSurface *s,const Radio *r)
{
    ui_clear(s,C_BG); ui_frequency(s,r,5);
    const char *state;
    if(g_scan.vfo_mode) state=g_scan.squelch_open?"VFO RX":"VFO HOLD";
    else if(scanner_is_hold(&g_scan)) state=g_scan.squelch_open?"HOLD RX":"HOLD";
    else state=g_scan.squelch_open?"RECEIVE":"SCAN";
    ui_textf(s,8,39,2,g_scan.squelch_open?C_OK:(scanner_is_scanning(&g_scan)?C_WARN:C_TEXT),"%s",state);
    if(g_scan.vfo_mode){
        ui_textf(s,8,65,2,C_TEXT,"STEP %.3f kHz",(double)g_step_hz[g_scan.vfo_step_index]/1000.0);
        ui_text(s,8,91,"LEFT/RIGHT TUNE",2,C_DIM); ui_text(s,8,110,"UP/DOWN STEP",2,C_DIM);
    } else {
        ScanBank *b=scanner_bank(&g_scan); ScanChannel *c=scanner_channel(&g_scan);
        if(b){ui_textf(s,8,62,1,C_DIM,"BANK %d/%d",g_scan.bank_index+1,g_scan.bank_count);ui_text(s,8,75,b->name,2,C_TEXT);}
        if(c){ui_textf(s,8,98,1,C_DIM,"CHANNEL %d/%d",g_scan.channel_index+1,b->channel_count);ui_text(s,8,111,c->name,2,C_TEXT);}
    }
    ui_text(s,8,145,"A SCAN/HOLD",2,C_TEXT);
    ui_text(s,8,164,"X DISPLAY",2,C_TEXT);
    ui_text(s,8,183,"Y MENU",2,C_TEXT);
    ui_text(s,166,145,"R AVOID",2,C_TEXT);
    ui_text(s,166,164,"L MONITOR",2,C_TEXT);
    ui_text(s,166,183,"ZL/ZR SQL",2,C_TEXT);
    ui_textf(s,8,215,1,C_DIM,"HOLD AUDIO %s",g_scan.hold_audio_monitor?"MONITOR":"SQUELCHED");
    if(scanner_is_scanning(&g_scan)){
        ui_textf(s,8,228,1,C_DIM,"VIEW: %s%s",
                 g_scan.scope_visible?(g_waterfall_visible?"WATERFALL":"SCOPE"):"RADIO",
                 g_scan.scope_visible?(g_scan.scope_screen?" (TOP)":" (BOTTOM)"):"");
    }
}

static void ui_menu(UiSurface *s,const Radio *r)
{
    ui_clear(s,C_BG);
    char f[32]; snprintf(f,sizeof(f),"%03lu.%06lu MHz",(unsigned long)(r->freq_hz/1000000U),(unsigned long)(r->freq_hz%1000000U));
    ui_center(s,4,f,2,C_ACCENT);
    char cat[48]; snprintf(cat,sizeof(cat),"< %s >",menu_cat_name(g_menu_cat)); ui_center(s,25,cat,2,C_WARN);
    int n=menu_item_count(g_menu_cat), first=0; if(g_menu_item>7)first=g_menu_item-7;
    char buf[64];
    for(int row=0;row<8&&first+row<n;row++){
        int item=first+row,y=53+row*20; menu_item_text(g_menu_cat,item,buf,sizeof(buf));
        if(item==g_menu_item)ui_rect(s,5,y-3,s->w-10,18,C_SELECT);
        ui_text(s,10,y,buf,2,item==g_menu_item?C_TEXT:C_DIM);
    }
    ui_text(s,8,218,"LEFT/RIGHT TAB  UP/DOWN ITEM",1,C_DIM);
    ui_text(s,8,230,"A SELECT   B/Y CLOSE",1,C_DIM);
}

static void ui_scope(UiSurface *s,const Radio *r)
{
    ui_clear(s,C_BG);
    int overlay=g_scan.scope_overlay;
    int top=(overlay?38:10), left=38, right=s->w-10, bottom=218;
    if(overlay)ui_frequency(s,r,4);
    static const int dbTicks[5]={-10,-20,-30,-40,-50};
    for(int n=0;n<5;n++){
        float frac=((float)FFT_DB_TOP-dbTicks[n])/(FFT_DB_TOP-FFT_DB_BOTTOM);
        int y=top+(int)(frac*(bottom-top)); ui_line(s,left,y,right,y,C_GRID);
        ui_textf(s,2,y-3,1,C_DIM,"%d",dbTicks[n]);
    }
    for(int n=0;n<5;n++){ int x=left+n*(right-left)/4; ui_line(s,x,top,x,bottom,n==2?C_SELECT:C_GRID); }
    if(fftSmoothInit){
        int prevx=left,prevy=bottom;
        for(int px=0;px<=right-left;px++){
            int bin=px*(FFT_N-1)/(right-left),bm=bin?bin-1:bin,bp=bin<FFT_N-1?bin+1:bin;
            float db=(fftSmooth[bm]+2.0f*fftSmooth[bin]+fftSmooth[bp])*0.25f;
            if(db>FFT_DB_TOP)db=FFT_DB_TOP;if(db<FFT_DB_BOTTOM)db=FFT_DB_BOTTOM;
            float norm=(db-FFT_DB_BOTTOM)/(FFT_DB_TOP-FFT_DB_BOTTOM); int x=left+px,y=bottom-(int)(norm*(bottom-top));
            if(px)ui_line(s,prevx,prevy,x,y,C_ACCENT); prevx=x;prevy=y;
        }
    }
    if(overlay>=2){
        ScanBank *b=scanner_bank(&g_scan); ScanChannel *c=scanner_channel(&g_scan);
        ui_rect(s,44,top+6,s->w-54,38,C_PANEL);
        if(g_scan.vfo_mode)ui_textf(s,50,top+12,1,C_TEXT,"VFO  %s  SQ %s",mode_name(r->mode),g_scan.squelch_open?"OPEN":"CLOSED");
        else { if(b)ui_text(s,50,top+10,b->name,1,C_TEXT); if(c)ui_text(s,50,top+22,c->name,1,C_TEXT); }
        ui_textf(s,50,top+32,1,C_DIM,"M %+.1f dB",(double)g_scan.margin_db);
    }
}

static void waterfall_push_smoothed_fft(void)
{
    u8 *row=g_waterfall[g_waterfall_head];
    for(int bin=0;bin<FFT_N;bin++){
        int bm=bin?bin-1:bin;
        int bp=bin<FFT_N-1?bin+1:bin;
        float db=(fftSmooth[bm]+2.0f*fftSmooth[bin]+fftSmooth[bp])*0.25f;
        if(db>FFT_DB_TOP) db=FFT_DB_TOP;
        if(db<FFT_DB_BOTTOM) db=FFT_DB_BOTTOM;
        float norm=(db-FFT_DB_BOTTOM)/(FFT_DB_TOP-FFT_DB_BOTTOM);
        int v=(int)(norm*255.0f+0.5f);
        if(v<0) v=0;
        if(v>255) v=255;
        row[bin]=(u8)v;
    }
    g_waterfall_head=(g_waterfall_head+1)%WATERFALL_ROWS;
    if(g_waterfall_count<WATERFALL_ROWS) g_waterfall_count++;
}

static UiColor waterfall_color(u8 v)
{
    /* Compact black -> blue -> cyan -> yellow -> red heat map. */
    UiColor c={0,0,0};
    if(v<64){
        c.b=(u8)(v*3);
    }else if(v<128){
        int t=v-64;
        c.g=(u8)(t*3);
        c.b=(u8)(192+t);
    }else if(v<192){
        int t=v-128;
        c.r=(u8)(t*3);
        c.g=255;
        c.b=(u8)(255-t*4);
    }else{
        int t=v-192;
        c.r=255;
        c.g=(u8)(255-t*4);
        c.b=0;
    }
    return c;
}

static void ui_waterfall(UiSurface *s,const Radio *r)
{
    ui_clear(s,C_BG);
    int overlay=g_scan.scope_overlay;
    int top=0, left=0, right=s->w-1, bottom=s->h-1;

    /* Full-screen waterfall: display geometry only.  The FFT bins, smoothing,
     * sample rate and RF span are unchanged; bins are mapped across the
     * available framebuffer width.  Vertical axis is time. */
    int plotH=bottom-top+1;
    if(plotH>WATERFALL_ROWS) plotH=WATERFALL_ROWS;
    int rows=g_waterfall_count<plotH?g_waterfall_count:plotH;
    for(int ry=0;ry<rows;ry++){
        int src=(g_waterfall_head-1-ry+WATERFALL_ROWS)%WATERFALL_ROWS;
        int y=top+ry;
        for(int x=left;x<=right;x++){
            int bin=(x-left)*(FFT_N-1)/(right-left);
            ui_pixel(s,x,y,waterfall_color(g_waterfall[src][bin]));
        }
    }

    /* Keep tuned center visible over the waterfall. */
    ui_line(s,(left+right)/2,top,(left+right)/2,bottom,C_SELECT);

    /* Existing scope overlays remain overlays; they no longer reserve plot
     * area, so the waterfall itself still occupies the complete screen. */
    if(overlay) ui_frequency(s,r,4);
    if(overlay>=2){
        ScanBank *b=scanner_bank(&g_scan); ScanChannel *c=scanner_channel(&g_scan);
        ui_rect(s,6,42,s->w-12,38,C_PANEL);
        if(g_scan.vfo_mode) ui_textf(s,12,48,1,C_TEXT,"VFO  %s  SQ %s",mode_name(r->mode),g_scan.squelch_open?"OPEN":"CLOSED");
        else { if(b)ui_text(s,12,46,b->name,1,C_TEXT); if(c)ui_text(s,12,58,c->name,1,C_TEXT); }
        ui_textf(s,12,68,1,C_DIM,"M %+.1f dB",(double)g_scan.margin_db);
    }
}

static void ui_debug(UiSurface *top,UiSurface *bottom,const Radio *r)
{
    ui_clear(top,C_BG); ui_clear(bottom,C_BG);
    ui_text(top,8,6,"V0.1.0-BETA DIAGNOSTICS",2,C_WARN);
    ui_textf(top,8,28,2,C_ACCENT,"%03lu.%06lu MHz %s",(unsigned long)(r->freq_hz/1000000U),(unsigned long)(r->freq_hz%1000000U),mode_name(r->mode));
    ui_textf(top,8,55,1,C_TEXT,"SOCKET %lu/s  10s %lu/s",(unsigned long)g_diag.sock1,(unsigned long)g_diag.sock10);
    ui_textf(top,8,70,1,C_TEXT,"ACCEPT %lu/s  10s %lu/s",(unsigned long)g_diag.accept1,(unsigned long)g_diag.accept10);
    ui_textf(top,8,85,1,C_TEXT,"DSP    %lu/s  10s %lu/s",(unsigned long)g_diag.dsp1,(unsigned long)g_diag.dsp10);
    ui_textf(top,8,105,1,C_TEXT,"IQ %lu%% [%lu-%lu]  DROP %.3f%%",(unsigned long)g_diag.iqPct,(unsigned long)g_diag.iqMinPct,(unsigned long)g_diag.iqMaxPct,g_diag.dropPct);
    ui_textf(top,8,120,1,C_TEXT,"PCM %lu -> %lu/s  SYNC %+ldppm",(unsigned long)g_diag.pcmSrc,(unsigned long)g_diag.pcmOut,g_diag.ppm);
    ui_textf(top,8,135,1,C_TEXT,"AUDIO %lu%% [%lu-%lu]",(unsigned long)g_diag.afPct,(unsigned long)g_diag.afMinPct,(unsigned long)g_diag.afMaxPct);
    ui_textf(top,8,150,1,C_DIM,"OVR IQ:%lu AF:%lu REBUF:%lu STARVE:%llums",
             (unsigned long)g_iq_overruns,(unsigned long)g_audio_overruns,
             (unsigned long)g_audio_rebuffers,(unsigned long long)current_starved_ms());
    {
        bool sqlOvr=false;
        int effSql=scanner_effective_sql(&g_scan,&sqlOvr);
        ui_textf(top,8,168,1,C_TEXT,"%s %s %s  SQL G:%d E:%d%s",
                 scanner_is_scanning(&g_scan)?"SCAN":"HOLD",
                 scanner_state_name(g_scan.state),
                 scan_resume_name(g_scan.resume_mode),
                 g_scan.sql_level,effSql,sqlOvr?" OVR":"");
    }
    ui_textf(top,8,183,1,C_TEXT,"RF %.1f NF %.1f M %+.1f  GATE %s",
             (double)g_scan.rf_db,(double)g_scan.noise_db,(double)g_scan.margin_db,
             g_audio_gate_open?"OPEN":"CLOSED");
    ui_textf(top,8,198,1,C_TEXT,"HITS %lu PASSES %lu ELIG %d/%d",
             (unsigned long)g_scan.hits,(unsigned long)g_scan.passes,
             scanner_eligible_count(&g_scan),scanner_total_channel_count(&g_scan));
    {
        ScanBank *db=scanner_bank(&g_scan);
        ui_textf(top,8,211,1,C_DIM,"BANK ELIG %d/%d",
                 scanner_bank_eligible_count(&g_scan,g_scan.bank_index),
                 db?db->channel_count:0);
    }
    ui_text(top,8,226,"SELECT RADIO   START EXIT",1,C_DIM);

    ui_text(bottom,8,8,"CURRENT",2,C_WARN); ui_frequency(bottom,r,30);
    ScanBank *b=scanner_bank(&g_scan); ScanChannel *c=scanner_channel(&g_scan);
    if(b)ui_text(bottom,8,68,b->name,2,C_TEXT); if(c)ui_text(bottom,8,90,c->name,2,C_TEXT);
    {
        bool sqlOvr=false;
        int effSql=scanner_effective_sql(&g_scan,&sqlOvr);
        ui_textf(bottom,8,120,2,C_TEXT,"SQ %s SQL %d%s",
                 g_scan.squelch_open?"OPEN":"CLOSED",effSql,sqlOvr?" OVR":"");
        ui_textf(bottom,8,137,1,C_DIM,"AUDIO GATE %s",g_audio_gate_open?"OPEN":"CLOSED");
    }
    ui_textf(bottom,8,154,2,C_TEXT,"IQ %lu%%",(unsigned long)g_diag.iqPct);
    ui_textf(bottom,8,177,2,C_TEXT,"AUDIO %lu%%",(unsigned long)g_diag.afPct);
    ui_textf(bottom,8,205,1,C_DIM,"DROPPED %llu BYTES",(unsigned long long)g_diag.droppedBytes);
}

static void ui_prepare_fft(void)
{
    if(!fftReady)return; fft_compute(); const float scaleDb=20.0f*log10f(2.0f/(float)FFT_N);
    for(int i=0;i<FFT_N;i++){ float db=fftMag[i]+scaleDb; if(!fftSmoothInit||fftRetunePending)fftSmooth[i]=db; else fftSmooth[i]+=FFT_SMOOTH_ALPHA*(db-fftSmooth[i]); }
    fftSmoothInit=true; fftRetunePending=false;
    waterfall_push_smoothed_fft();
    fftFill=0; fftReady=false;
}

static void ui_present(void)
{ gfxFlushBuffers(); gfxSwapBuffersGpu(); gspWaitForVBlank(); }

static void ui_frequency_editor_bottom(UiSurface *bottom,const char *title,u32 freq,int digit);

static void ui_render(const Radio *r,u32 iqPct,u32 afPct)
{
    UiSurface top=ui_surface(GFX_TOP), bottom=ui_surface(GFX_BOTTOM);

    if(g_freq_edit_active && g_freq_edit_target){
        /* Keep the top screen truthful: it always shows the live receiver.
         * Only the bottom screen becomes the memory/VFO digit editor. */
        ui_radio_status(&top,r,iqPct,afPct);
        ui_frequency_editor_bottom(&bottom,g_freq_edit_title,
                                   *g_freq_edit_target,g_freq_edit_digit);
        ui_present();
        g_ui_dirty=false;
        g_last_ui_render_ms=osGetTime();
        return;
    }


    /* The user's selected view is persistent.  During idle scanning we keep
     * both radio/status screens visible so the changing channel is readable;
     * once a carrier is received, or whenever scanning is held/stopped, the
     * selected scope/waterfall view is shown again. */
    bool scanIdle=scanner_is_scanning(&g_scan)&&!g_scan.squelch_open;
    bool showScope=g_scan.scope_visible&&g_scope_allowed&&!scanIdle;
    bool showWaterfall=showScope&&g_waterfall_visible;
    if(showScope)ui_prepare_fft();
    if(g_show_debug) ui_debug(&top,&bottom,r);
    else if(showScope&&g_scan.scope_screen==1){ if(showWaterfall)ui_waterfall(&top,r);else ui_scope(&top,r); if(g_menu_open)ui_menu(&bottom,r);else ui_control_status(&bottom,r); }
    else if(showScope&&g_scan.scope_screen==0){ if(g_menu_open)ui_menu(&top,r);else ui_radio_status(&top,r,iqPct,afPct); if(showWaterfall)ui_waterfall(&bottom,r);else ui_scope(&bottom,r); }
    else { ui_radio_status(&top,r,iqPct,afPct); if(g_menu_open)ui_menu(&bottom,r);else ui_control_status(&bottom,r); }
    ui_present();
    g_ui_dirty=false; g_last_ui_render_ms=osGetTime();
}

static bool server_ip_parse_octets(const char *host,u8 oct[4])
{
    unsigned a=0,b=0,c=0,d=0; char tail=0;
    if(sscanf(host,"%u.%u.%u.%u%c",&a,&b,&c,&d,&tail)!=4) return false;
    if(a>255||b>255||c>255||d>255) return false;
    oct[0]=(u8)a; oct[1]=(u8)b; oct[2]=(u8)c; oct[3]=(u8)d;
    return true;
}

static void server_ip_adjust_digit(u8 oct[4],int digit,int dir)
{
    static const int place[3]={100,10,1};
    if(digit<0||digit>11) return;
    int oi=digit/3, di=digit%3, p=place[di];
    int old=(oct[oi]/p)%10;

    /* Cycle only through decimal digits that leave the selected octet in the
     * legal IPv4 range.  This keeps the frequency-editor style while making
     * it impossible to construct 256..999. */
    for(int step=1;step<=10;step++){
        int nd=(old + (dir>0?step:-step))%10;
        if(nd<0) nd+=10;
        int candidate=(int)oct[oi]-old*p+nd*p;
        if(candidate>=0 && candidate<=255){
            oct[oi]=(u8)candidate;
            return;
        }
    }
}

static void ui_server_ip_editor(int digit,const u8 oct[4])
{
    UiSurface top=ui_surface(GFX_TOP),bottom=ui_surface(GFX_BOTTOM);
    ui_clear(&top,C_BG); ui_clear(&bottom,C_BG);

    ui_center(&top,54,"RTL_TCP SERVER",3,C_ACCENT);
    ui_center(&top,104,"SELECT SERVER BEFORE CONNECTING",1,C_TEXT);
    ui_center(&top,126,"NO CONNECTION HAS BEEN STARTED",1,C_DIM);

    ui_center(&bottom,16,"ENTER IP",2,C_WARN);
    char ip[32];
    snprintf(ip,sizeof(ip),"%03u.%03u.%03u.%03u",
             (unsigned)oct[0],(unsigned)oct[1],
             (unsigned)oct[2],(unsigned)oct[3]);
    const int scale=3;
    int x=(bottom.w-ui_text_width(ip,scale))/2;
    ui_text(&bottom,x,66,ip,scale,C_ACCENT);

    int charIndex=digit + digit/3;
    int caretX=x+charIndex*6*scale+2*scale;
    ui_text(&bottom,caretX,94,"^",2,C_WARN);

    ui_center(&bottom,126,"LEFT/RIGHT SELECT DIGIT",1,C_TEXT);
    ui_center(&bottom,146,"UP/DOWN CHANGE",1,C_TEXT);
    ui_center(&bottom,176,"A CONNECT",2,C_OK);
    ui_center(&bottom,202,"PORT 1234",1,C_DIM);
    ui_center(&bottom,220,"START = EXIT",1,C_DIM);
    ui_present();
}

static bool startup_server_select(void)
{
    u8 oct[4]={10,0,0,15};
    if(!server_ip_parse_octets(g_server_host,oct))
        server_ip_parse_octets(DEFAULT_HOST,oct);

    int digit=0;
    ui_server_ip_editor(digit,oct);

    while(aptMainLoop()){
        hidScanInput();
        u32 down=hidKeysDown();
        if(down&KEY_START) return false;
        if(down&KEY_DLEFT){ if(digit>0) digit--; }
        if(down&KEY_DRIGHT){ if(digit<11) digit++; }
        if(down&KEY_DUP) server_ip_adjust_digit(oct,digit,+1);
        if(down&KEY_DDOWN) server_ip_adjust_digit(oct,digit,-1);
        if(down&KEY_A){
            snprintf(g_server_host,sizeof(g_server_host),"%u.%u.%u.%u",
                     (unsigned)oct[0],(unsigned)oct[1],
                     (unsigned)oct[2],(unsigned)oct[3]);
            server_host_save();
            return true;
        }
        if(down&(KEY_DLEFT|KEY_DRIGHT|KEY_DUP|KEY_DDOWN))
            ui_server_ip_editor(digit,oct);
        gspWaitForVBlank();
    }
    return false;
}

static void ui_message(const char *title,const char *line1,const char *line2)
{
    UiSurface top=ui_surface(GFX_TOP),bottom=ui_surface(GFX_BOTTOM); ui_clear(&top,C_BG);ui_clear(&bottom,C_BG);
    ui_center(&top,60,title,3,C_ACCENT); if(line1)ui_center(&top,110,line1,2,C_TEXT); if(line2)ui_center(&top,138,line2,2,C_DIM);
    ui_center(&bottom,96,"3DS SDR SCANNER",2,C_WARN); ui_center(&bottom,125,"START = EXIT",2,C_DIM); ui_present();
}

static void ui_frequency_editor_bottom(UiSurface *bottom,const char *title,u32 freq,int digit)
{
    ui_clear(bottom,C_BG);

    ui_center(bottom,16,title,2,C_WARN);

    char digits[24];
    snprintf(digits,sizeof(digits),"%03lu.%06lu",
             (unsigned long)(freq/1000000U),
             (unsigned long)(freq%1000000U));

    /* 3x is deliberately large enough to be readable while fitting the
     * complete 9-digit frequency on the 320-pixel lower LCD. */
    int scale=3;
    int x=(bottom->w-ui_text_width(digits,scale))/2;
    ui_text(bottom,x,66,digits,scale,C_ACCENT);

    int charIndex=digit+(digit>=3?1:0);
    int caretX=x+charIndex*6*scale+2*scale;
    ui_text(bottom,caretX,94,"^",2,C_WARN);

    ui_center(bottom,126,"LEFT/RIGHT SELECT DIGIT",1,C_TEXT);
    ui_center(bottom,146,"UP/DOWN CHANGE",1,C_TEXT);
    ui_center(bottom,176,"A SAVE    B CANCEL",2,C_DIM);

    ScanBank *b=edit_bank();
    ScanChannel *c=edit_channel();
    if(!g_freq_edit_live_vfo && b && c){
        ui_center(bottom,210,b->name,1,C_DIM);
        ui_center(bottom,224,c->name,1,C_DIM);
    }else if(g_freq_edit_live_vfo){
        ui_center(bottom,218,"LIVE VFO",1,C_DIM);
    }
}


/* Drain bytes that are already queued on the client socket before issuing a
 * scanner retune.  rtl_tcp is a continuous untagged IQ stream; without this
 * barrier, queued samples from the previous channel can be attributed to the
 * next memory after the UI/state has already advanced.  This is nonblocking
 * and bounded so a busy stream cannot stall the main loop indefinitely. */
static u64 radio_drain_queued_iq(int sock)
{
    if(sock < 0) return 0;

    u8 scratch[4096];
    u64 dropped=0;
    const u64 cap=1048576ULL;

    while(dropped < cap){
        int n=recv(sock,scratch,sizeof(scratch),0);
        if(n>0){
            dropped += (u64)n;
            continue;
        }
        if(n==0) break;
        if(errno==EAGAIN || errno==EWOULDBLOCK ||
           errno==EINPROGRESS || errno==EALREADY)
            break;
        break;
    }
    return dropped;
}

static u64 flush_initial_iq(int sock, u8 *scratch, u32 scratchBytes, u32 durationMs)
{
    u64 start = osGetTime();
    u64 dropped = 0;

    while (aptMainLoop() && osGetTime() - start < durationMs) {
        hidScanInput();
        if (hidKeysDown() & KEY_START)
            break;

        bool got = false;
        for (int k = 0; k < 16; k++) {
            int n = recv(sock, scratch, scratchBytes, 0);
            if (n > 0) {
                dropped += (u64)n;
                got = true;
                continue;
            }
            if (n == 0)
                return dropped;

            if (errno == EAGAIN || errno == EWOULDBLOCK ||
                errno == EINPROGRESS || errno == EALREADY)
                break;

            return dropped;
        }

        if (!got)
            svcSleepThread(1000000); // 1 ms
    }

    return dropped;
}

static void radio_retune_reset(Radio *r)
{
    // Discard all old-frequency IQ waiting in the software jitter buffer.
    iq_fifo_reset();

    // Old demodulated PCM is equally stale after a retune.
    ndspChnReset(0);
    ndspChnSetInterp(0, NDSP_INTERP_LINEAR);
    ndspChnSetRate(0, (float)AUDIO_RATE);
    ndspChnSetFormat(0, NDSP_FORMAT_MONO_PCM16);
    memset(waveBuf, 0, sizeof(waveBuf));
    audio_fifo_reset();

    // Reset FM/filter history so phase/filter state from the previous RF
    // channel cannot bleed into the newly tuned channel.
    r->prev_i = 0.0f;
    r->prev_q = 0.0f;
    r->box_count = 0;
    r->audio_count = 0;
    r->audio_accum = 0.0f;
    r->deemph_y = 0.0f;
    memset(r->wbfm_fir, 0, sizeof(r->wbfm_fir));
    r->wbfm_fir_pos = 0;
    r->wbfm_decim_phase = 0;

    // Drop any partially captured FFT, but keep the last completed trace visible
    // while the buffered DSP path refills after a retune. The first completed
    // FFT from the new buffered stream replaces it directly.
    fftFill = 0;
    fftReady = false;
    fftRetunePending = true;

    scanner_channel_metric_reset();
    scanner_scanfast_metric_reset();
    g_scan.rf_db = -60.0f;
    g_scan.noise_db = -60.0f;
    g_scan.margin_db = 0.0f;
    g_scan.squelch_open = false;
    g_scan.sql_candidate_open = false;
    g_scan.sql_candidate_ms = osGetTime();

    /* Do not let untagged old-frequency rtl_tcp samples cross a retune.
     * NFM scanner operation gets a longer quarantine because it makes fast
     * carrier decisions directly from socket IQ; WFM keeps its existing
     * public-beta timing. */
    u32 discard_ms=(r->mode==MODE_NFM)?NFM_RX_RETUNE_DISCARD_MS:RX_RETUNE_DISCARD_MS;
    g_rx_discard_until_ms = osGetTime() + discard_ms;
}

static bool radio_reconnect(Radio *r)
{
    u32 keep_freq=r->freq_hz;
    RadioMode keep_mode=r->mode;

    if(r->sock>=0){
        close(r->sock);
        r->sock=-1;
    }

    /* A few short attempts are enough for transient SOC/applet resets without
     * trapping the user forever if rtl_tcp itself is actually down. */
    int s=-1;
    for(int attempt=0;attempt<3 && aptMainLoop();attempt++){
        s=connect_rtl(g_server_host,DEFAULT_PORT);
        if(s>=0) break;
        svcSleepThread(250000000LL); /* 250 ms */
    }
    if(s<0) return false;

    r->sock=s;
    r->freq_hz=keep_freq;

    /* Re-issue the rtl_tcp operating state onto the new connection. */
    radio_set_mode(r,keep_mode);
    radio_retune_reset(r);

    r->recv_bytes=0;
    r->recv_calls=0;
    scanner_ui_dirty();
    return true;
}


static void scanner_tune_current(Scanner *s, Radio *r)
{
    ScanChannel *c = scanner_channel(s);
    if (!c) return;

    /* First empty the 3DS-side receive queue while it still unambiguously
     * belongs to the old channel.  After the rtl_tcp retune command, the
     * normal producer quarantine below drains in-flight/server-buffered IQ
     * before detector, scope, or audio are allowed to consume it. */
    (void)radio_drain_queued_iq(r->sock);

    r->freq_hz = c->freq_hz;
    if (r->mode != c->mode) {
        radio_set_mode(r, c->mode);
        radio_retune_reset(r);
    } else {
        send_rtl_cmd(r->sock, 0x01, r->freq_hz);
        radio_retune_reset(r);
    }

    s->squelch_open = false;
    s->sql_candidate_open = false;
    s->sql_candidate_ms = osGetTime();
    s->state = scanner_is_scanning(s) ? SCANNER_SETTLE : SCANNER_STOPPED;
    s->state_since_ms = osGetTime();

    if(scanner_is_scanning(s)){
        g_audio_gate_open = false;
        g_scope_allowed = false;
    }else{
        g_audio_gate_open = s->monitor || s->hold_audio_monitor;
        g_scope_allowed = true;
    }
    scanner_ui_dirty();
}

static void scanner_start(Scanner *s, Radio *r)
{
    s->run_mode = SCANNER_RUN_SCAN;
    s->scanning = true;
    s->state = SCANNER_STOPPED;
    s->state_since_ms = osGetTime();

    if (!scanner_channel_eligible(s, s->bank_index, s->channel_index)) {
        if (!scanner_advance(s, +1)) {
            s->run_mode = SCANNER_RUN_HOLD;
            s->scanning = false;
            s->state = SCANNER_STOPPED;
            scanner_ui_dirty();
            return;
        }
    }
    scanner_tune_current(s, r);
}

static void scanner_stop(Scanner *s)
{
    /* HOLD is a hard operating mode, not merely a paused scan state. */
    s->run_mode = SCANNER_RUN_HOLD;
    s->scanning = false;
    s->state = SCANNER_STOPPED;
    s->state_since_ms = osGetTime();

    /* Cancel any pending scan transition/delay and evaluate audio only from
     * the held channel's squelch/monitor state. */
    s->sql_candidate_open = s->squelch_open;
    s->sql_candidate_ms = osGetTime();
    g_audio_gate_open = s->monitor || s->hold_audio_monitor || s->squelch_open;
    g_scope_allowed = true;
    scanner_ui_dirty();
}

static void scanner_next(Scanner *s, Radio *r, int dir)
{
    /* Automatic channel movement is forbidden while HOLD is active. */
    if (!scanner_is_scanning(s))
        return;

    if (!scanner_advance(s, dir))
        return;

    /* scanner_advance may have stopped scanning if nothing is eligible. */
    if (!scanner_is_scanning(s))
        return;

    scanner_tune_current(s, r);
}

static void scanner_tick(Scanner *s, Radio *r, u64 now)
{
    /* Scanner control and playback are deliberately separate for NFM.
     * While NFM SCAN is active, fresh post-retune rtl_tcp IQ is authoritative
     * for carrier state through CHECK, RECEIVE, and DELAY.  The shallow
     * buffered NFM DSP path remains responsible only for audio/scope playback.
     * WFM keeps the existing buffered squelch/playback behavior unchanged. */
    bool nfm_live_scan = !s->vfo_mode && scanner_is_scanning(s) &&
                         r->mode==MODE_NFM &&
                         (s->state==SCANNER_CHECK ||
                          s->state==SCANNER_RECEIVE ||
                          s->state==SCANNER_DELAY);
    if(nfm_live_scan)
        scanner_scanfast_update_squelch(s,now);
    else if(!(scanner_is_scanning(s) && r->mode==MODE_NFM &&
              s->state==SCANNER_SETTLE))
        scanner_update_squelch(s,now);

    if (s->vfo_mode || scanner_is_hold(s)) {
        /* HOLD may update squelch/audio status, but it can never enter the
         * scan state machine or advance away from the selected channel. */
        s->scanning = false;
        s->state = SCANNER_STOPPED;
        bool force_monitor = s->monitor || s->hold_audio_monitor;
        g_audio_gate_open = force_monitor || s->squelch_open;
        g_scope_allowed = true;
        return;
    }

    /* Keep legacy/UI mirror coherent with the explicit run mode. */
    s->scanning = true;

    switch (s->state) {
        case SCANNER_SETTLE:
            g_audio_gate_open = false;
            g_scope_allowed = false;
            /* Do not start the settle clock until the post-retune socket
             * quarantine has ended.  During the quarantine the producer is
             * intentionally draining untagged old-frequency IQ. */
            if(now < g_rx_discard_until_ms)
                break;
            if(s->state_since_ms < g_rx_discard_until_ms)
                s->state_since_ms = g_rx_discard_until_ms;
            if (now - s->state_since_ms >= SCAN_SETTLE_MS) {
                /* Keep the post-discard scan-fast history gathered during
                 * SETTLE.  The detector was already reset by the retune path,
                 * and preserving these fresh samples lets CHECK start with an
                 * established carrier decision instead of starting cold. */
                s->state = SCANNER_CHECK;
                s->state_since_ms = now;
            }
            break;

        case SCANNER_CHECK:
            if (s->squelch_open) {
                s->state = SCANNER_RECEIVE;
                s->state_since_ms = now;
                g_audio_gate_open = true;
                g_scope_allowed = true;
            } else if (now - s->state_since_ms >= SCAN_CHECK_MS) {
                /* Give acquisition enough real post-retune time to make a
                 * trustworthy decision.  This deliberately favors reliable
                 * capture over maximum scan speed; ~340 ms total dwell is
                 * still roughly three idle channels per second. */
                s->passes++;
                scanner_next(s, r, +1);
            }
            break;

        case SCANNER_RECEIVE: {
            g_audio_gate_open = s->monitor || s->squelch_open;
            g_scope_allowed = s->squelch_open;

            if (s->resume_mode == SCAN_RESUME_TIME &&
                now - s->state_since_ms >= s->time_hold_ms) {
                scanner_next(s, r, +1);
                break;
            }

            if (!s->squelch_open) {
                ScanChannel *c = scanner_channel(s);
                if (s->resume_mode == SCAN_RESUME_CARRIER) {
                    scanner_next(s, r, +1);
                } else {
                    s->state = SCANNER_DELAY;
                    s->state_since_ms = now;
                    g_scope_allowed = true; // leave last trace visible during delay
                    g_audio_gate_open = false;
                    if (!c) scanner_next(s, r, +1);
                }
            }
            break;
        }

        case SCANNER_DELAY: {
            ScanChannel *c = scanner_channel(s);
            u32 delay = c ? c->delay_ms : 2000;
            g_audio_gate_open = false;
            g_scope_allowed = true;

            // A reply during the delay reopens the channel immediately.
            if (s->squelch_open) {
                s->state = SCANNER_RECEIVE;
                s->state_since_ms = now;
                g_audio_gate_open = true;
                g_scope_allowed = true;
            } else if (now - s->state_since_ms >= delay) {
                scanner_next(s, r, +1);
            }
            break;
        }

        default:
            scanner_next(s, r, +1);
            break;
    }
}

static void scanner_cycle_resume(Scanner *s)
{
    s->resume_mode = (ScanResumeMode)(((int)s->resume_mode + 1) % 3);
    scanner_save(s);
    scanner_ui_dirty();
}

static void scanner_adjust_sql(Scanner *s, int delta)
{
    s->sql_level += delta;
    if (s->sql_level < 0) s->sql_level = 0;
    if (s->sql_level > 100) s->sql_level = 100;
    scanner_save(s);
    scanner_ui_dirty();
}

static bool scanner_keyboard_edit(Radio *r,char *dst,size_t dstsz,const char *hint)
{
    SwkbdState kb;
    char tmp[SCAN_NAME_LEN];
    snprintf(tmp,sizeof(tmp),"%s",dst);

    swkbdInit(&kb,SWKBD_TYPE_NORMAL,2,(int)dstsz-1);
    swkbdSetHintText(&kb,hint);
    swkbdSetInitialText(&kb,tmp);
    SwkbdButton b=swkbdInputText(&kb,tmp,sizeof(tmp));

    /* Applet exit can leave A/B reported for another frame.  Never let the
     * confirmation press fall through into the menu and trigger the same
     * action again; the main loop ignores all keys until everything is up. */
    g_input_lock_until_release = true;

    /* The applet blocks our producer/DSP loop. Missed audio is acceptable.
     * Replace the rtl_tcp session entirely on return: this prevents a socket
     * that was reset/stalled during the applet from poisoning later edits. */
    radio_reconnect(r);

    if(b!=SWKBD_BUTTON_CONFIRM){
        g_ui_dirty=true;
        return false;
    }

    snprintf(dst,dstsz,"%s",tmp);
    g_ui_dirty=true;
    return true;
}

static void frequency_editor_begin(const char *title,u32 *freq,bool live_vfo)
{
    if(!freq) return;
    g_freq_edit_active=true;
    g_freq_edit_target=freq;
    g_freq_edit_original=*freq;
    g_freq_edit_digit=3;
    g_freq_edit_live_vfo=live_vfo;
    snprintf(g_freq_edit_title,sizeof(g_freq_edit_title),"%s",title);
    scanner_ui_dirty();
}

static bool frequency_editor_handle(u32 down,Radio *r)
{
    if(!g_freq_edit_active || !g_freq_edit_target) return false;

    if(down&KEY_A){
        bool live=g_freq_edit_live_vfo;
        g_freq_edit_active=false;
        if(live && g_scan.vfo_mode)
            tune_frequency(r,g_scan.vfo_freq_hz,g_scan.vfo_radio_mode);
        scanner_save(&g_scan);
        scanner_ui_dirty();
        return true;
    }

    if(down&KEY_B){
        *g_freq_edit_target=g_freq_edit_original;
        g_freq_edit_active=false;
        scanner_ui_dirty();
        return true;
    }

    if((down&KEY_DLEFT) && g_freq_edit_digit>0) g_freq_edit_digit--;
    if((down&KEY_DRIGHT) && g_freq_edit_digit<8) g_freq_edit_digit++;

    if(down&(KEY_DUP|KEY_DDOWN)){
        static const u32 place[9]={100000000U,10000000U,1000000U,100000U,10000U,1000U,100U,10U,1U};
        u32 p=place[g_freq_edit_digit],v=*g_freq_edit_target,n=(v/p)%10U;
        if(down&KEY_DUP) v=(n<9U)?v+p:v-9U*p;
        else v=(n>0U)?v-p:v+9U*p;
        if(v>=100000U && v<=999999999U) *g_freq_edit_target=v;
        scanner_ui_dirty();
    }
    return true;
}

static void tune_frequency(Radio *r, u32 freq, RadioMode mode)
{
    r->freq_hz=freq;
    if(r->mode!=mode) {
        radio_set_mode(r,mode);
        radio_retune_reset(r);
    } else {
        send_rtl_cmd(r->sock,0x01,r->freq_hz);
        radio_retune_reset(r);
    }
    g_scan.squelch_open=false; g_scan.sql_candidate_open=false; g_scan.sql_candidate_ms=osGetTime();
    scanner_ui_dirty();
}

static void enter_vfo(Radio *r)
{
    g_scan.vfo_mode=true; g_scan.scanning=false; g_scan.state=SCANNER_STOPPED;
    tune_frequency(r,g_scan.vfo_freq_hz,g_scan.vfo_radio_mode);
}

static void leave_vfo(Radio *r)
{
    g_scan.vfo_mode=false; g_scan.scanning=false; g_scan.state=SCANNER_STOPPED;
    ScanChannel *c=scanner_channel(&g_scan);
    if(c) tune_frequency(r,c->freq_hz,c->mode);
}

static void clear_temp_avoids(void)
{
    for(int bi=0;bi<g_scan.bank_count;bi++) for(int ci=0;ci<g_scan.banks[bi].channel_count;ci++) g_scan.banks[bi].channels[ci].temp_avoid=false;
    scanner_ui_dirty();
}

static void add_bank(Radio *r)
{
    if(g_scan.bank_count>=SCAN_MAX_BANKS) return;
    ScanBank *b=&g_scan.banks[g_scan.bank_count++];
    memset(b,0,sizeof(*b));
    b->enabled=true;
    snprintf(b->name,sizeof(b->name),"New Bank");
    g_edit_bank_index=g_scan.bank_count-1;
    g_edit_channel_index=0;
    scanner_keyboard_edit(r,b->name,sizeof(b->name),"Bank name");
    scanner_save(&g_scan);
    scanner_ui_dirty();
}

static void delete_edit_bank(void)
{
    if(g_scan.bank_count<=1) return;
    if(g_edit_bank_index==g_scan.bank_index) return; /* keep live memory stable */

    int bi=g_edit_bank_index;
    for(int i=bi;i<g_scan.bank_count-1;i++) g_scan.banks[i]=g_scan.banks[i+1];
    g_scan.bank_count--;

    /* Preserve the live memory selection when an earlier bank shifts down. */
    if(g_scan.bank_index>bi) g_scan.bank_index--;
    if(g_edit_bank_index>=g_scan.bank_count) g_edit_bank_index=g_scan.bank_count-1;
    g_edit_channel_index=0;

    scanner_save(&g_scan);
    scanner_ui_dirty();
}

static bool channel_exact_duplicate(const ScanChannel *a,const ScanChannel *b)
{
    if(!a || !b) return false;
    return a->freq_hz==b->freq_hz &&
           a->mode==b->mode &&
           a->enabled==b->enabled &&
           a->priority==b->priority &&
           a->delay_ms==b->delay_ms &&
           a->step_index==b->step_index &&
           a->squelch_override==b->squelch_override &&
           strcmp(a->name,b->name)==0;
}

static bool channel_frequency_mode_duplicate(const ScanChannel *a,const ScanChannel *b)
{
    if(!a || !b) return false;
    return a->freq_hz==b->freq_hz && a->mode==b->mode;
}

static void add_channel(Radio *r,bool from_vfo)
{
    ScanBank *b=edit_bank();
    if(!b || b->channel_count>=SCAN_MAX_CHANNELS) return;

    /* Build the new memory off-list first.  The bank is not mutated until the
     * keyboard is confirmed, so cancelling the applet cannot leave a ghost
     * "New Channel" behind. */
    ScanChannel draft;
    memset(&draft,0,sizeof(draft));
    ScanChannel *template=edit_channel();

    if(from_vfo){
        draft.freq_hz=g_scan.vfo_freq_hz;
        draft.mode=g_scan.vfo_radio_mode;
    }else if(template){
        draft.freq_hz=template->freq_hz;
        draft.mode=template->mode;
        draft.step_index=template->step_index;
    }else{
        draft.freq_hz=146520000U;
        draft.mode=MODE_NFM;
        draft.step_index=5;
    }

    draft.enabled=true;
    draft.delay_ms=2000;
    if(draft.step_index<0 || draft.step_index>=STEP_COUNT) draft.step_index=5;
    draft.squelch_override=-1;
    snprintf(draft.name,sizeof(draft.name),from_vfo?"VFO Memory":"New Channel");

    if(!scanner_keyboard_edit(r,draft.name,sizeof(draft.name),"Channel name")){
        scanner_ui_dirty();
        return;
    }

    /* Conservative duplicate protection: reject only an exact duplicate.
     * Same-frequency memories with intentionally different names/settings are
     * still allowed. */
    for(int i=0;i<b->channel_count;i++){
        if(channel_exact_duplicate(&draft,&b->channels[i])){
            g_edit_channel_index=i;
            if(g_menu_cat==MENU_CHANNEL) g_menu_item=1;
            else if(g_menu_cat==MENU_BANKS) g_menu_item=0;
            else if(g_menu_cat==MENU_RADIO) g_menu_item=0;
            scanner_ui_dirty();
            return;
        }
    }

    b->channels[b->channel_count]=draft;
    g_edit_channel_index=b->channel_count;
    b->channel_count++;

    /* Do not leave the menu cursor sitting on an Add/Save action.  Together
     * with the post-keyboard release lock this makes a repeated add require a
     * deliberate navigation back to the action. */
    if(g_menu_cat==MENU_CHANNEL) g_menu_item=1;
    else if(g_menu_cat==MENU_BANKS) g_menu_item=0;
    else if(g_menu_cat==MENU_RADIO) g_menu_item=0;

    scanner_save(&g_scan);

    /* New-memory workflow: after the channel name is confirmed and the
     * memory is committed, continue directly into the existing frequency
     * editor for that new memory.  Ordinary Edit Name remains unchanged. */
    frequency_editor_begin("EDIT MEMORY FREQUENCY",
                           &b->channels[g_edit_channel_index].freq_hz,false);
    scanner_ui_dirty();
}

static void delete_edit_channel(void)
{
    ScanBank *b=edit_bank();
    if(!b || b->channel_count<=0) return;

    /* Do not invalidate the memory that currently labels the live receiver. */
    if(g_edit_bank_index==g_scan.bank_index &&
       g_edit_channel_index==g_scan.channel_index)
        return;

    int ci=g_edit_channel_index;
    for(int i=ci;i<b->channel_count-1;i++) b->channels[i]=b->channels[i+1];
    b->channel_count--;

    if(g_edit_bank_index==g_scan.bank_index && g_scan.channel_index>ci)
        g_scan.channel_index--;

    if(g_edit_channel_index>=b->channel_count) g_edit_channel_index=b->channel_count-1;
    if(g_edit_channel_index<0) g_edit_channel_index=0;

    scanner_save(&g_scan);
    scanner_ui_dirty();
}

static int remove_duplicate_channels_in_bank(int bi)
{
    if(bi<0 || bi>=g_scan.bank_count) return 0;

    ScanBank *b=&g_scan.banks[bi];
    int removed=0;
    for(int i=0;i<b->channel_count;i++){
        for(int j=i+1;j<b->channel_count;){
            if(channel_frequency_mode_duplicate(&b->channels[i],&b->channels[j])){
                if(g_scan.bank_index==bi){
                    if(g_scan.channel_index==j) g_scan.channel_index=i;
                    else if(g_scan.channel_index>j) g_scan.channel_index--;
                }
                if(g_edit_bank_index==bi){
                    if(g_edit_channel_index==j) g_edit_channel_index=i;
                    else if(g_edit_channel_index>j) g_edit_channel_index--;
                }

                for(int k=j;k<b->channel_count-1;k++)
                    b->channels[k]=b->channels[k+1];
                b->channel_count--;
                removed++;
            }else{
                j++;
            }
        }
    }

    if(g_scan.bank_index==bi){
        if(b->channel_count<=0) g_scan.channel_index=0;
        else if(g_scan.channel_index>=b->channel_count) g_scan.channel_index=b->channel_count-1;
    }
    if(g_edit_bank_index==bi){
        if(b->channel_count<=0) g_edit_channel_index=0;
        else if(g_edit_channel_index>=b->channel_count) g_edit_channel_index=b->channel_count-1;
    }
    return removed;
}

static int remove_duplicate_channels_all_banks(void)
{
    int removed=0;
    for(int bi=0;bi<g_scan.bank_count;bi++)
        removed+=remove_duplicate_channels_in_bank(bi);

    if(removed>0) scanner_save(&g_scan);
    scanner_ui_dirty();
    return removed;
}

static int remove_duplicate_channels_current_bank(void)
{
    int removed=remove_duplicate_channels_in_bank(g_scan.bank_index);
    if(removed>0) scanner_save(&g_scan);
    scanner_ui_dirty();
    return removed;
}

static int menu_item_count(MenuCategory c)
{
    static const int n[]={12,6,5,6,3,5};
    return n[(int)c];
}

static const char *menu_cat_name(MenuCategory c)
{
    static const char *n[]={"CHANNEL","BANKS","SCAN","RADIO","DISPLAY","SYSTEM"};
    return n[(int)c];
}

static void menu_item_text(MenuCategory cat,int item,char *buf,size_t n)
{
    ScanBank *b=edit_bank();
    ScanChannel *c=edit_channel();

    switch(cat){
    case MENU_CHANNEL:
        switch(item){
        case 0:
            snprintf(buf,n,"Edit Bank: %d/%d %s",
                     g_edit_bank_index+1,g_scan.bank_count,b?b->name:"--");
            break;
        case 1:
            snprintf(buf,n,"Edit Ch: %d/%d %s",
                     c?g_edit_channel_index+1:0,b?b->channel_count:0,
                     c?c->name:"--");
            break;
        case 2: snprintf(buf,n,"Edit Name"); break;
        case 3: snprintf(buf,n,"Edit Frequency"); break;
        case 4: snprintf(buf,n,"Mode: %s",c?mode_name(c->mode):"--"); break;
        case 5: snprintf(buf,n,"Scan Enabled: %s",c&&c->enabled?"ON":"OFF"); break;
        case 6: snprintf(buf,n,"Priority: %s",c&&c->priority?"ON":"OFF"); break;
        case 7: snprintf(buf,n,"Delay: %.1f sec",c?(double)c->delay_ms/1000.0:0.0); break;
        case 8:
            if(c&&c->squelch_override>=0) snprintf(buf,n,"Squelch: %d",c->squelch_override);
            else snprintf(buf,n,"Squelch: GLOBAL");
            break;
        case 9: snprintf(buf,n,"Step: %.3f kHz",c?(double)g_step_hz[c->step_index]/1000.0:0.0); break;
        case 10: snprintf(buf,n,"Add Channel"); break;
        default: snprintf(buf,n,"Delete Channel"); break;
        }
        break;

    case MENU_BANKS:
        switch(item){
        case 0: snprintf(buf,n,"Edit Bank: %d/%d %s",g_edit_bank_index+1,g_scan.bank_count,b?b->name:"--"); break;
        case 1: snprintf(buf,n,"Rename Bank"); break;
        case 2: snprintf(buf,n,"Bank Enabled: %s",b&&b->enabled?"ON":"OFF"); break;
        case 3: snprintf(buf,n,"Add Bank"); break;
        case 4: snprintf(buf,n,"Delete Bank"); break;
        default: snprintf(buf,n,"Add Channel"); break;
        }
        break;

    case MENU_SCAN:
        switch(item){
        case 0: snprintf(buf,n,"Resume: %s",scan_resume_name(g_scan.resume_mode)); break;
        case 1: snprintf(buf,n,"Global SQL: %d",g_scan.sql_level); break;
        case 2: snprintf(buf,n,"Time Hold: %.1f sec",(double)g_scan.time_hold_ms/1000.0); break;
        case 3: snprintf(buf,n,"Clear Temp Avoids"); break;
        default: snprintf(buf,n,"%s",scanner_is_scanning(&g_scan)?"HOLD":"START SCAN"); break;
        }
        break;

    case MENU_RADIO:
        switch(item){
        case 0: snprintf(buf,n,"Mode: %s",g_scan.vfo_mode?"VFO":"CHANNEL"); break;
        case 1: snprintf(buf,n,"Edit VFO Frequency"); break;
        case 2: snprintf(buf,n,"VFO Mode: %s",mode_name(g_scan.vfo_radio_mode)); break;
        case 3: snprintf(buf,n,"VFO Step: %.3f kHz",(double)g_step_hz[g_scan.vfo_step_index]/1000.0); break;
        case 4: snprintf(buf,n,"Save VFO To Edit Bank"); break;
        default: snprintf(buf,n,"Hold Audio: %s",g_scan.hold_audio_monitor?"MONITOR":"SQUELCHED"); break;
        }
        break;

    case MENU_DISPLAY:
        switch(item){
        case 0:
            snprintf(buf,n,"View: %s",g_scan.scope_visible?(g_waterfall_visible?"WATERFALL":"SCOPE"):"RADIO");
            break;
        case 1: snprintf(buf,n,"Scope Screen: %s",g_scan.scope_screen?"TOP":"BOTTOM"); break;
        default: {
            static const char *ov[]={"OFF","MINIMAL","FULL"};
            snprintf(buf,n,"Scope Overlay: %s",ov[g_scan.scope_overlay]);
            break;
        }}
        break;

    default:
        if(item==0) snprintf(buf,n,"Save Settings");
        else if(item==1) snprintf(buf,n,"Reset Hit Counters");
        else if(item==2) snprintf(buf,n,"Remove Dups Current Bank");
        else if(item==3) snprintf(buf,n,"Remove Dups All Banks");
        else snprintf(buf,n,"Diagnostics");
        break;
    }
}

static void menu_action(Radio *r)
{
    ScanBank *b=edit_bank();
    ScanChannel *c=edit_channel();

    switch(g_menu_cat){
    case MENU_CHANNEL:
        switch(g_menu_item){
        case 0:
            edit_cycle_bank(+1);
            break;
        case 1:
            edit_cycle_channel(+1);
            break;
        case 2:
            if(c) scanner_keyboard_edit(r,c->name,sizeof(c->name),"Channel name");
            break;
        case 3:
            if(c) frequency_editor_begin("EDIT MEMORY FREQUENCY",&c->freq_hz,false);
            break;
        case 4:
            if(c) c->mode=(c->mode==MODE_NFM)?MODE_WBFM:MODE_NFM;
            break;
        case 5:
            if(c) c->enabled=!c->enabled;
            break;
        case 6:
            if(c) c->priority=!c->priority;
            break;
        case 7:
            if(c){
                static const u32 d[]={0,1000,2000,3000,5000};
                int k=0;
                while(k<5&&d[k]!=c->delay_ms)k++;
                c->delay_ms=d[(k+1)%5];
            }
            break;
        case 8:
            if(c){
                if(c->squelch_override<0)c->squelch_override=20;
                else if(c->squelch_override>=100)c->squelch_override=-1;
                else c->squelch_override+=10;
            }
            break;
        case 9:
            if(c)c->step_index=(c->step_index+1)%STEP_COUNT;
            break;
        case 10:
            add_channel(r,false);
            break;
        case 11:
            delete_edit_channel();
            break;
        }
        scanner_save(&g_scan);
        break;

    case MENU_BANKS:
        switch(g_menu_item){
        case 0: edit_cycle_bank(+1); break;
        case 1: if(b)scanner_keyboard_edit(r,b->name,sizeof(b->name),"Bank name"); break;
        case 2: if(b)b->enabled=!b->enabled; break;
        case 3: add_bank(r); break;
        case 4: delete_edit_bank(); break;
        case 5: add_channel(r,false); break;
        }
        scanner_save(&g_scan);
        break;

    case MENU_SCAN:
        switch(g_menu_item){
        case 0: scanner_cycle_resume(&g_scan); break;
        case 1:
            scanner_adjust_sql(&g_scan,5);
            if(g_scan.sql_level>=100)g_scan.sql_level=0;
            break;
        case 2:
            if(g_scan.time_hold_ms==2000)g_scan.time_hold_ms=5000;
            else if(g_scan.time_hold_ms==5000)g_scan.time_hold_ms=10000;
            else g_scan.time_hold_ms=2000;
            break;
        case 3: clear_temp_avoids(); break;
        case 4:
            if(scanner_is_scanning(&g_scan))scanner_stop(&g_scan);
            else if(!g_scan.vfo_mode)scanner_start(&g_scan,r);
            break;
        }
        scanner_save(&g_scan);
        break;

    case MENU_RADIO:
        switch(g_menu_item){
        case 0:
            if(g_scan.vfo_mode)leave_vfo(r);
            else enter_vfo(r);
            break;
        case 1:
            frequency_editor_begin("EDIT VFO FREQUENCY",&g_scan.vfo_freq_hz,true);
            break;
        case 2:
            g_scan.vfo_radio_mode=(g_scan.vfo_radio_mode==MODE_NFM)?MODE_WBFM:MODE_NFM;
            if(g_scan.vfo_mode)tune_frequency(r,g_scan.vfo_freq_hz,g_scan.vfo_radio_mode);
            break;
        case 3:
            g_scan.vfo_step_index=(g_scan.vfo_step_index+1)%STEP_COUNT;
            break;
        case 4:
            if(g_scan.vfo_mode) add_channel(r,true);
            break;
        case 5:
            g_scan.hold_audio_monitor=!g_scan.hold_audio_monitor;
            break;
        }
        scanner_save(&g_scan);
        break;

    case MENU_DISPLAY:
        switch(g_menu_item){
        case 0:
            if(!g_scan.scope_visible){
                g_scan.scope_visible=true;
                g_waterfall_visible=false;
            }else if(!g_waterfall_visible){
                g_waterfall_visible=true;
            }else{
                g_scan.scope_visible=false;
                g_waterfall_visible=false;
            }
            break;
        case 1: g_scan.scope_screen=g_scan.scope_screen?0:1; break;
        default: g_scan.scope_overlay=(g_scan.scope_overlay+1)%3; break;
        }
        scanner_save(&g_scan);
        break;

    case MENU_SYSTEM:
        if(g_menu_item==0) scanner_save(&g_scan);
        else if(g_menu_item==1){
            for(int bi=0;bi<g_scan.bank_count;bi++)
                for(int ci=0;ci<g_scan.banks[bi].channel_count;ci++)
                    g_scan.banks[bi].channels[ci].hit_count=0;
            g_scan.hits=0;
        }else if(g_menu_item==2){
            remove_duplicate_channels_current_bank();
        }else if(g_menu_item==3){
            remove_duplicate_channels_all_banks();
        }else g_show_debug=true;
        break;

    default:
        break;
    }
    scanner_ui_dirty();
}

int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    gfxInitDefault();
    gfxSetDoubleBuffering(GFX_TOP, true);
    gfxSetDoubleBuffering(GFX_BOTTOM, true);
    g_ptmu_ready = R_SUCCEEDED(ptmuInit());
    if(g_ptmu_ready) battery_poll();

    server_host_load();
    if(!startup_server_select()) goto done_gfx;

    char backendLine[48];
    snprintf(backendLine,sizeof(backendLine),"%s:%d",g_server_host,DEFAULT_PORT);
    ui_message("V0.1.0-BETA","CONNECTING RADIO BACKEND",backendLine);

    // New 3DS: enable the higher CPU clock/L2 when available.
    osSetSpeedupEnable(true);

    socBuffer = (u32*)memalign(SOC_ALIGN, SOC_SIZE);
    if (!socBuffer || socInit(socBuffer, SOC_SIZE) != 0) {
        ui_message("NETWORK ERROR","socInit FAILED","PRESS START TO EXIT");
        goto done_gfx;
    }

    if (audio_init() != 0) {
        ui_message("AUDIO ERROR","ndspInit FAILED","PRESS START TO EXIT");
        goto done_soc;
    }

    // Initialize cleanup/diagnostic state before any later goto target can
    // observe it.
    bool unexpectedStop = false;
    char stopReason[96] = {0};
    u8 *iqbuf = NULL;
    u8 *iqProcessBuf = NULL;

    Radio r;
    memset(&r, 0, sizeof(r));
    r.sock = connect_rtl(g_server_host, DEFAULT_PORT);
    r.freq_hz = DEFAULT_FREQ_HZ;
    if (r.sock < 0) goto done_audio;

    scanner_load(&g_scan);
    ScanChannel *bootChannel = scanner_channel(&g_scan);
    if (bootChannel) {
        r.freq_hz = bootChannel->freq_hz;
        radio_set_mode(&r, bootChannel->mode);
    } else {
        radio_set_mode(&r, MODE_WBFM);
    }


    iqbuf = (u8*)malloc(IQ_READ_BYTES);
    iqProcessBuf = (u8*)malloc(IQ_READ_BYTES);
    iqFifo = (u8*)malloc(IQ_FIFO_BYTES);

    if (!iqbuf || !iqProcessBuf || !iqFifo) {
        ui_message("MEMORY ERROR","IQ BUFFER ALLOCATION FAILED","PRESS START TO EXIT");
        goto loop_end;
    }

    // Discard a short startup backlog after rtl_tcp has accepted our
    // sample-rate/frequency/gain commands. This keeps old socket backlog out of
    // the jitter-buffer accounting.
    u64 startupDiscardBytes = flush_initial_iq(r.sock, iqbuf, IQ_READ_BYTES, 250);

    iq_fifo_reset();
    r.recv_bytes = 0;
    r.recv_calls = 0;

    scanner_stop(&g_scan);
    g_show_debug = false;
    ui_render(&r,0,0);

    u64 lastStatusMs = osGetTime();
    u32 lastRecvBytes = 0;
    u64 lastAcceptedBytes = 0;
    u64 lastIqDsp = 0;
    u64 lastPcmSource = 0;
    u64 lastPcmOutput = 0;

    // Ten-second accounting window.
    u64 avgWindowStartMs = lastStatusMs;
    u64 avgSockStartBytes = 0;
    u64 avgAcceptedStartBytes = 0;
    u64 avgDspStartPairs = 0;
    u32 sock10 = 0;
    u32 accept10 = 0;
    u32 dsp10 = 0;

    // Cumulative loss is measured from the clean post-flush start.
    u64 totalSocketBytes = 0;
    (void)startupDiscardBytes;

    // Lightweight application-load meter: accumulate time spent actively in
    // this main loop and compare it with total ARM11 system ticks.  This is
    // main-thread busy percentage, not the APT CPU-time-limit setting.
    u64 cpuBusyTicks = 0;
    u64 cpuWindowStartTick = svcGetSystemTick();

    while (aptMainLoop()) {
        u64 loopBusyStartTick = svcGetSystemTick();
        hidScanInput();
        u32 down = hidKeysDown();
        u32 held = hidKeysHeld();

        /* swkbd can return with the confirm/cancel key still logically held.
         * Ignore every input until all keys have been released once. */
        if(g_input_lock_until_release){
            down = 0;
            if(held==0) g_input_lock_until_release=false;
        }

        if (down & KEY_START) break;

        if (down & KEY_SELECT) {
            g_menu_open = false;
            g_show_debug = !g_show_debug;
            scanner_ui_dirty();
        }

        g_scan.monitor = (held & KEY_L) != 0;

        // Y opens/closes the radio-firmware menu. While open, D-pad/A belong
        // exclusively to the menu rather than to bank/channel navigation.
        if (!g_freq_edit_active && (down & KEY_Y)) {
            bool opening = !g_menu_open;
            g_menu_open = opening;
            g_menu_item = 0;
            g_ui_dirty = true;
            if(opening){
                edit_cursor_from_live();
                if(scanner_is_scanning(&g_scan)) scanner_stop(&g_scan);
            }
            scanner_ui_dirty();
        }

        if (g_freq_edit_active) {
            frequency_editor_handle(down,&r);
        } else if (g_menu_open) {
            if (down & KEY_B) { g_menu_open = false; g_ui_dirty = true; }
            if (down & KEY_DLEFT) { g_menu_cat = (MenuCategory)((g_menu_cat + MENU_CAT_COUNT - 1) % MENU_CAT_COUNT); g_menu_item = 0; }
            if (down & KEY_DRIGHT){ g_menu_cat = (MenuCategory)((g_menu_cat + 1) % MENU_CAT_COUNT); g_menu_item = 0; }
            if (down & KEY_DUP) { if (g_menu_item > 0) g_menu_item--; }
            if (down & KEY_DDOWN) { int n=menu_item_count(g_menu_cat); if (g_menu_item < n-1) g_menu_item++; }
            if (down & KEY_A) menu_action(&r);
        } else {
            if (down & KEY_B) {
                u64 toggleNow=osGetTime();
                if(g_receiver_running){
                    g_receiver_running=false;
                    g_receiver_resuming_until_ms=0;
                    g_receiver_paused_at_ms=toggleNow;
                    g_audio_gate_open=false;
                    ndspChnReset(0);
                    memset(waveBuf,0,sizeof(waveBuf));
                    audio_fifo_reset();
                    iq_fifo_reset();
                }else{
                    u64 pausedFor=g_receiver_paused_at_ms?toggleNow-g_receiver_paused_at_ms:0;
                    if(g_scan.state_since_ms) g_scan.state_since_ms+=pausedFor;
                    if(g_scan.sql_candidate_ms) g_scan.sql_candidate_ms+=pausedFor;
                    g_receiver_paused_at_ms=0;
                    g_receiver_running=true;
                    radio_retune_reset(&r);
                    g_receiver_resuming_until_ms=g_rx_discard_until_ms;
                }
                scanner_ui_dirty();
            }

            if (down & KEY_A) {
                if (!g_scan.vfo_mode) {
                    if (scanner_is_scanning(&g_scan)) scanner_stop(&g_scan); else scanner_start(&g_scan,&r);
                }
            }

            // X cycles the display page: normal -> scope -> waterfall -> normal.
            if (down & KEY_X) {
                if(!g_scan.scope_visible){
                    g_scan.scope_visible=true;
                    g_waterfall_visible=false;
                }else if(!g_waterfall_visible){
                    g_waterfall_visible=true;
                }else{
                    g_scan.scope_visible=false;
                    g_waterfall_visible=false;
                }
                scanner_save(&g_scan);
                scanner_ui_dirty();
            }

            if (down & KEY_R && !g_scan.vfo_mode) {
                ScanChannel *c=scanner_channel(&g_scan);
                if(c){ c->temp_avoid=true; scanner_ui_dirty();
                    if(scanner_is_scanning(&g_scan)){ if(!scanner_advance(&g_scan,+1))scanner_stop(&g_scan); else scanner_tune_current(&g_scan,&r); }
                }
            }

            if (g_scan.vfo_mode) {
                u32 step=g_step_hz[g_scan.vfo_step_index];
                if (down & KEY_DLEFT) { if(g_scan.vfo_freq_hz>step)g_scan.vfo_freq_hz-=step; tune_frequency(&r,g_scan.vfo_freq_hz,g_scan.vfo_radio_mode); }
                if (down & KEY_DRIGHT){ g_scan.vfo_freq_hz+=step; tune_frequency(&r,g_scan.vfo_freq_hz,g_scan.vfo_radio_mode); }
                if (down & KEY_DUP) { g_scan.vfo_step_index=(g_scan.vfo_step_index+1)%STEP_COUNT; scanner_save(&g_scan); }
                if (down & KEY_DDOWN){ g_scan.vfo_step_index=(g_scan.vfo_step_index+STEP_COUNT-1)%STEP_COUNT; scanner_save(&g_scan); }
            } else {
                // Scanner hierarchy: Up/Down bank, Left/Right channel.
                if (down & KEY_DUP) scanner_select_bank(&g_scan,&r,-1);
                if (down & KEY_DDOWN) scanner_select_bank(&g_scan,&r,+1);
                if (down & KEY_DLEFT) scanner_select_channel_in_bank(&g_scan,&r,-1);
                if (down & KEY_DRIGHT) scanner_select_channel_in_bank(&g_scan,&r,+1);
            }

            if (down & KEY_ZR) scanner_adjust_sql(&g_scan,+2);
            if (down & KEY_ZL) scanner_adjust_sql(&g_scan,-2);
        }

        // Producer: drain currently available TCP bytes into the raw-IQ ring.
        for (int k = 0; k < 16; k++) {
            int n = recv(r.sock, iqbuf, IQ_READ_BYTES, 0);

            if (n > 0) {
                r.recv_calls++;
                r.recv_bytes += (u32)n;
                if(g_receiver_running) totalSocketBytes += (u64)n;

                if(!g_receiver_running || osGetTime() < g_rx_discard_until_ms){
                    /* While stopped, or just after a retune/resume, deliberately
                     * drain rtl_tcp but do not let stale IQ enter the DSP FIFO. */
                    continue;
                }

                /* During NFM SCAN, carrier control follows the newest IQ
                 * directly rather than the delayed playback FIFO.  Continue
                 * feeding the live detector after acquisition so RECEIVE can
                 * remain locked to an active carrier and CARRIER/DELAY resume
                 * behavior reflects live RF rather than buffered history. */
                if(r.mode==MODE_NFM && !g_scan.vfo_mode &&
                   scanner_is_scanning(&g_scan) &&
                   (g_scan.state==SCANNER_SETTLE ||
                    g_scan.state==SCANNER_CHECK ||
                    g_scan.state==SCANNER_RECEIVE ||
                    g_scan.state==SCANNER_DELAY))
                    scanner_scanfast_feed(iqbuf,(u32)n);

                iq_fifo_write_bytes_mode(iqbuf, (u32)n, r.mode);
            } else if (n == 0) {
                if(radio_reconnect(&r)) {
                    lastRecvBytes = r.recv_bytes;
                    lastAcceptedBytes = g_iq_accepted_bytes;
                    lastIqDsp = g_iq_dsp_pairs;
                    lastPcmSource = g_pcm_source_samples;
                    lastPcmOutput = g_pcm_output_samples;
                    lastStatusMs = osGetTime();
                    break;
                }

                unexpectedStop = true;
                snprintf(stopReason, sizeof(stopReason),
                         "rtl_tcp disconnected; reconnect failed");
                goto loop_end;
            } else {
                if (errno == EAGAIN || errno == EWOULDBLOCK ||
                    errno == EINPROGRESS || errno == EALREADY)
                    break;

                if (errno == ECONNRESET || errno == ENOTCONN ||
                    errno == EPIPE || errno == ETIMEDOUT) {
                    if(radio_reconnect(&r)) {
                        lastRecvBytes = r.recv_bytes;
                        lastAcceptedBytes = g_iq_accepted_bytes;
                        lastIqDsp = g_iq_dsp_pairs;
                        lastPcmSource = g_pcm_source_samples;
                        lastPcmOutput = g_pcm_output_samples;
                        lastStatusMs = osGetTime();
                        break;
                    }
                }

                unexpectedStop = true;
                snprintf(stopReason, sizeof(stopReason),
                         "recv error: errno=%d", errno);
                goto loop_end;
            }
        }

        // Consumer: demodulate according to paced sample-clock credit, not TCP
        // packet arrival timing. Master STOP leaves the socket alive but freezes
        // DSP/audio/scanner progression.
        if(g_receiver_running){
            for (int k = 0; k < 4; k++) {
                if (iq_process_paced(&r, iqProcessBuf, IQ_READ_BYTES) == 0)
                    break;
                audio_pump();
            }

            audio_pump();
        }else{
            g_audio_gate_open=false;
        }

        u64 nowMs = osGetTime();
        if(g_receiver_running) scanner_tick(&g_scan, &r, nowMs);
        u64 elapsedMs = nowMs - lastStatusMs;
        if (elapsedMs >= 1000) {
            u32 bytesDelta = r.recv_bytes - lastRecvBytes;
            u64 acceptedDelta = g_iq_accepted_bytes - lastAcceptedBytes;
            u64 iqDspDelta = g_iq_dsp_pairs - lastIqDsp;
            u64 srcDelta = g_pcm_source_samples - lastPcmSource;
            u64 outDelta = g_pcm_output_samples - lastPcmOutput;

            // Report sample rates, not byte rates, for direct comparison.
            u32 sock1 = elapsedMs
                ? (u32)(((u64)bytesDelta * 1000ULL) / (elapsedMs * 2ULL))
                : 0;
            u32 accept1 = elapsedMs
                ? (u32)((acceptedDelta * 1000ULL) / (elapsedMs * 2ULL))
                : 0;
            u32 dsp1 = elapsedMs
                ? (u32)((iqDspDelta * 1000ULL) / elapsedMs)
                : 0;
            u32 srcPerSec = elapsedMs
                ? (u32)((srcDelta * 1000ULL) / elapsedMs)
                : 0;
            u32 outPerSec = elapsedMs
                ? (u32)((outDelta * 1000ULL) / elapsedMs)
                : 0;

            lastRecvBytes = r.recv_bytes;
            lastAcceptedBytes = g_iq_accepted_bytes;
            lastIqDsp = g_iq_dsp_pairs;
            lastPcmSource = g_pcm_source_samples;
            lastPcmOutput = g_pcm_output_samples;
            lastStatusMs = nowMs;

            // Refresh a true multi-second average every ten seconds.
            u64 avgElapsed = nowMs - avgWindowStartMs;
            if (avgElapsed >= 10000) {
                u64 sockBytes10 = (u64)r.recv_bytes - avgSockStartBytes;
                u64 acceptBytes10 = g_iq_accepted_bytes - avgAcceptedStartBytes;
                u64 dspPairs10 = g_iq_dsp_pairs - avgDspStartPairs;

                sock10 = (u32)((sockBytes10 * 1000ULL) / (avgElapsed * 2ULL));
                accept10 = (u32)((acceptBytes10 * 1000ULL) / (avgElapsed * 2ULL));
                dsp10 = (u32)((dspPairs10 * 1000ULL) / avgElapsed);

                avgWindowStartMs = nowMs;
                avgSockStartBytes = r.recv_bytes;
                avgAcceptedStartBytes = g_iq_accepted_bytes;
                avgDspStartPairs = g_iq_dsp_pairs;
            }

            u32 afPct = (audioFifoCount * 100U) / AUDIO_FIFO_SAMPLES;
            u32 afMinPct = (g_fifo_window_min * 100U) / AUDIO_FIFO_SAMPLES;
            u32 afMaxPct = (g_fifo_window_max * 100U) / AUDIO_FIFO_SAMPLES;

            u32 iqPct = (iqFifoCount * 100U) / IQ_FIFO_BYTES;
            u32 iqMinPct = (g_iq_window_min * 100U) / IQ_FIFO_BYTES;
            u32 iqMaxPct = (g_iq_window_max * 100U) / IQ_FIFO_BYTES;

            float outputRatio = 1.0f / g_resamp_step;
            long ppm = (long)((outputRatio - 1.0f) * 1000000.0f);

            u64 droppedBytes = (totalSocketBytes >= g_iq_accepted_bytes)
                ? totalSocketBytes - g_iq_accepted_bytes
                : 0;
            double dropPct = totalSocketBytes
                ? (100.0 * (double)droppedBytes / (double)totalSocketBytes)
                : 0.0;

            u64 cpuNowTick = svcGetSystemTick();
            u64 cpuWindowTicks = cpuNowTick - cpuWindowStartTick;
            u32 cpuPct = cpuWindowTicks
                ? (u32)((cpuBusyTicks * 100ULL) / cpuWindowTicks)
                : 0;
            if(cpuPct > 100U) cpuPct = 100U;
            cpuBusyTicks = 0;
            cpuWindowStartTick = cpuNowTick;

            struct mallinfo memInfo = mallinfo();
            u32 linearFreeBytes = linearSpaceFree();
            u64 memFreeBytes = (u64)(memInfo.fordblks > 0 ? memInfo.fordblks : 0)
                             + (u64)linearFreeBytes;
            u64 memUsedBytes = (u64)(memInfo.uordblks > 0 ? memInfo.uordblks : 0);
            if(__ctru_linear_heap_size >= linearFreeBytes)
                memUsedBytes += (u64)(__ctru_linear_heap_size - linearFreeBytes);
            u32 memUsedMb = (u32)(memUsedBytes / (1024ULL*1024ULL));
            u32 memFreeMb = (u32)(memFreeBytes / (1024ULL*1024ULL));

            g_diag.sock1=sock1; g_diag.sock10=sock10;
            g_diag.accept1=accept1; g_diag.accept10=accept10;
            g_diag.dsp1=dsp1; g_diag.dsp10=dsp10;
            g_diag.iqPct=iqPct; g_diag.iqMinPct=iqMinPct; g_diag.iqMaxPct=iqMaxPct;
            g_diag.afPct=afPct; g_diag.afMinPct=afMinPct; g_diag.afMaxPct=afMaxPct;
            g_diag.cpuPct=cpuPct; g_diag.memUsedMb=memUsedMb; g_diag.memFreeMb=memFreeMb;
            g_diag.ppm=ppm; g_diag.droppedBytes=droppedBytes; g_diag.dropPct=dropPct;
            g_diag.pcmSrc=srcPerSec; g_diag.pcmOut=outPerSec;
            if(g_show_debug) g_ui_dirty=true;

            g_fifo_window_min = audioFifoCount;
            g_fifo_window_max = audioFifoCount;
            g_iq_window_min = iqFifoCount;
            g_iq_window_max = iqFifoCount;
        }

        // One compositor owns both framebuffers. Text/status uses a relaxed
        // 5 Hz cadence; scope uses the existing 25 Hz FFT cadence.
        bool scanIdle=scanner_is_scanning(&g_scan)&&!g_scan.squelch_open;
        bool showScope=g_scan.scope_visible&&g_scope_allowed&&!scanIdle;
        u64 uiInterval=showScope?FFT_REFRESH_MS:TEXT_UI_REFRESH_MS;
        if(g_ui_dirty || nowMs-g_last_ui_render_ms>=uiInterval){
            u32 iqPctNow=(iqFifoCount*100U)/IQ_FIFO_BYTES;
            u32 afPctNow=(audioFifoCount*100U)/AUDIO_FIFO_SAMPLES;
            ui_render(&r,iqPctNow,afPctNow);
        }

        // Count active main-loop time before the cooperative yield so CPU is
        // a useful application-load metric rather than the configured limit.
        cpuBusyTicks += svcGetSystemTick() - loopBusyStartTick;

        // Yield briefly only when the socket has nothing available.
        svcSleepThread(250000); // 0.25 ms cooperative yield
    }

loop_end:
    scanner_save(&g_scan);
    if (iqbuf) free(iqbuf);
    if (iqProcessBuf) free(iqProcessBuf);

    if (iqFifo) {
        free(iqFifo);
        iqFifo = NULL;
    }

    if (r.sock >= 0) {
        close(r.sock);
        r.sock = -1;
    }

    if (unexpectedStop) {
        ndspChnReset(0);
        ui_message("RADIO STREAM STOPPED",stopReason,"PRESS START TO EXIT");
        while(aptMainLoop()){ hidScanInput(); if(hidKeysDown()&KEY_START)break; gspWaitForVBlank(); }
    }

done_audio:
    ndspChnReset(0);
    if (audioMem) linearFree(audioMem);
    ndspExit();
done_soc:
    socExit();
    if (socBuffer) free(socBuffer);
done_gfx:
    if(g_ptmu_ready){ ptmuExit(); g_ptmu_ready=false; }
    gfxExit();
    return 0;
}

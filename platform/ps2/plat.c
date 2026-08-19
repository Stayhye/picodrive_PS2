/*
 * PicoDrive platform interface for PS2
 * (Real-Time ADPCM BGM Engine - Safe Path Resolver)
 */

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <stdlib.h>
#include <malloc.h>
#include <errno.h>

#include <kernel.h>
#include <iopcontrol.h>
#include <sbv_patches.h>
#include <sifrpc.h>
#include <loadfile.h>
#include <ps2_filesystem_driver.h>
#include <ps2_joystick_driver.h>
#include <ps2_audio_driver.h>
#include <audsrv.h>
#include <libcdvd.h>

#ifdef ALIGNED
#undef ALIGNED
#endif
#include "../libpicofe/plat.h"
#include "../../pico/pico_int.h"

extern unsigned char audsrv_irx[];
extern unsigned int size_audsrv_irx;
extern unsigned char libsd_irx[];
extern unsigned int size_libsd_irx;

extern void *_gp;

static int bgm_running = 0;
static int bgm_tid = -1;
static int audsrv_initialized = 0;
static unsigned char bgm_stack[0x10000] __attribute__((aligned(16)));

static int sound_rates[] = { 11025, 22050, 44100, -1 };
struct plat_target plat_target = { .sound_rates = sound_rates };

/* -------------------------------------------------------------------- */
/* SPU2 ADPCM Coefficients                                             */
/* -------------------------------------------------------------------- */
static const double f[5][2] = {
    { 0.0, 0.0 },
    { 60.0 / 64.0, 0.0 },
    { 115.0 / 64.0, -52.0 / 64.0 },
    { 98.0 / 64.0, -55.0 / 64.0 },
    { 122.0 / 64.0, -60.0 / 64.0 }
};

static void decode_adpcm_frame(const unsigned char *chunk, short *out_pcm, double *s1, double *s2) {
    int predict_nr = chunk[0] >> 4;
    int shift_factor = chunk[0] & 0xf;

    if (predict_nr > 4) predict_nr = 0;

    for (int i = 0; i < 28; i++) {
        unsigned char byte = chunk[2 + (i >> 1)];
        int sample = (i & 1) ? (byte >> 4) : (byte & 0x0f);
        if (sample >= 8) sample -= 16;

        double dsample = (double)(sample << (12 - shift_factor));
        double s_0 = dsample + (*s1 * f[predict_nr][0]) + (*s2 * f[predict_nr][1]);

        *s1 = *s2;
        *s2 = s_0;

        int pcm = (int)s_0;
        if (pcm > 32767) pcm = 32767;
        if (pcm < -32768) pcm = -32768;

        out_pcm[i] = (short)pcm;
    }
}

/* Helper function to find existing ADP file across CD and Memory Card */
static FILE *open_menu_adp(void) {
    static const char *paths[] = {
        "cdfs:/SKIN/MENU.ADP;1",
        "cdfs:/SKIN/MENU.ADP",
        "cdfs:/skin/menu.adp;1",
        "cdfs:/skin/menu.adp",
        "cdfs:/MENU.ADP;1",
        "cdfs:/MENU.ADP",
        "mc0:/PICO/MENU.ADP",
        "mc1:/PICO/MENU.ADP",
        NULL
    };

    for (int i = 0; paths[i] != NULL; i++) {
        FILE *f = fopen(paths[i], "rb");
        if (f) {
            printf("[BGM] Found audio file at: %s\n", paths[i]);
            return f;
        }
    }

    printf("[BGM] ERROR: Could not find MENU.ADP in any skin directory.\n");
    return NULL;
}

/* -------------------------------------------------------------------- */
/* BGM Thread                                                           */
/* -------------------------------------------------------------------- */
static void bgm_thread_func(void *arg) {
    /* Wait 1.5s so disc spin-up and UI finish */
    usleep(1500000);

    if (!bgm_running) {
        bgm_tid = -1;
        ExitDeleteThread();
        return;
    }

    /* Initialize audsrv asynchronously */
    if (!audsrv_initialized) {
        if (audsrv_init() != 0) {
            bgm_tid = -1;
            bgm_running = 0;
            ExitDeleteThread();
            return;
        }
        audsrv_set_volume(MAX_VOLUME);
        audsrv_initialized = 1;
    }

    FILE *f = open_menu_adp();
    if (!f) {
        bgm_tid = -1;
        bgm_running = 0;
        ExitDeleteThread();
        return;
    }

    /* Target format: 44.1kHz Mono 16-bit PCM */
    int sample_rate = 44100;
    struct audsrv_fmt_t adp_fmt;
    adp_fmt.bits = 16;
    adp_fmt.freq = sample_rate; 
    adp_fmt.channels = 1;
    audsrv_set_format(&adp_fmt);

    /* Check and skip VAG header if present */
    char header_check[4];
    if (fread(header_check, 1, 4, f) == 4) {
        if (memcmp(header_check, "VAGp", 4) == 0) {
            fseek(f, 48, SEEK_SET); // Skip 48-byte VAG header
        } else {
            fseek(f, 0, SEEK_SET);  // Raw ADP file
        }
    }

    #define ADPCM_BLOCK_SIZE 2048 // 128 frames
    static unsigned char raw_adpcm[ADPCM_BLOCK_SIZE];
    static short pcm_buffer[128 * 28];
    double s1 = 0.0, s2 = 0.0;

    while (bgm_running) {
        int bytes_read = (int)fread(raw_adpcm, 1, ADPCM_BLOCK_SIZE, f);
        if (bytes_read < 16) {
            fseek(f, 48, SEEK_SET); // Loop track
            s1 = 0.0;
            s2 = 0.0;
            continue;
        }

        if (!bgm_running) break;

        /* Decode ADPCM frames to PCM */
        int frames = bytes_read / 16;
        for (int i = 0; i < frames; i++) {
            decode_adpcm_frame(&raw_adpcm[i * 16], &pcm_buffer[i * 28], &s1, &s2);
        }

        int total_samples = frames * 28;
        int pcm_bytes = total_samples * sizeof(short);

        /* Real-time speed control based on sample count */
        unsigned int chunk_duration_us = (unsigned int)(((long long)total_samples * 1000000LL) / sample_rate);

        audsrv_play_audio((char *)pcm_buffer, pcm_bytes);
        usleep(chunk_duration_us);
    }

    fclose(f);
    bgm_tid = -1;
    ExitDeleteThread();
}

void plat_start_bgm(void) {
    if (bgm_running || bgm_tid >= 0) return;
    
    ee_thread_t thread;
    memset(&thread, 0, sizeof(ee_thread_t));
    bgm_running = 1;

    thread.func = (void *)bgm_thread_func;
    thread.stack = bgm_stack;
    thread.stack_size = sizeof(bgm_stack);
    thread.initial_priority = 48;
    thread.gp_reg = &_gp;

    bgm_tid = CreateThread(&thread);
    if (bgm_tid >= 0) {
        StartThread(bgm_tid, NULL);
    } else {
        bgm_running = 0;
    }
}

void plat_stop_bgm(void) {
    if (!bgm_running) return;
    bgm_running = 0;
    
    int timeout = 50; 
    while (bgm_tid != -1 && timeout-- > 0) {
        usleep(1000);
    }
    
    sceCdStop();
    sceCdSync(0);
}

static void reset_IOP() {
    SifInitRpc(0);
#if !defined(DEBUG) || defined(BUILD_FOR_PCSX2)
    while (!SifIopReset(NULL, 0)) {};
#endif
    while (!SifIopSync()) {};
    SifInitRpc(0);
    sbv_patch_enable_lmb();
    sbv_patch_disable_prefix_check();
}

int plat_target_init(void) { return 0; }

void plat_target_finish(void) {
    plat_stop_bgm();
    if (audsrv_initialized) audsrv_quit();
    deinit_ps2_filesystem_driver();
}

int plat_parse_arg(int argc, char *argv[], int *x) { return 1; }

void plat_early_init(void) {
    reset_IOP();
    init_ps2_filesystem_driver();

    int ret;
    SifExecModuleBuffer(libsd_irx, size_libsd_irx, 0, NULL, &ret);
    SifExecModuleBuffer(audsrv_irx, size_audsrv_irx, 0, NULL, &ret);
}

int plat_get_root_dir(char *dst, int len) {    
    strncpy(dst, "mc0:/PICO/", (size_t)len);
    DIR *dir;
    if ((dir = opendir(dst))) closedir(dir);
    else mkdir(dst, 0777);
    return (int)strlen(dst);
}

int plat_get_skin_dir(char *dst, int len) {
    if (len > 5) strcpy(dst, "cdfs:/SKIN/");
    else if (len > 0) *dst = 0;
    return (int)strlen(dst);
}

int plat_get_data_dir(char *dst, int len) {
    if (len > 5) strcpy(dst, "cdfs:/ROMS/");
    else if (len > 0) *dst = 0;
    return (int)strlen(dst);
}

int plat_is_dir(const char *path) {
    DIR *dir;
    if ((dir = opendir(path))) { closedir(dir); return 1; }
    return 0;
}

unsigned int plat_get_ticks_ms(void) { return (unsigned int)(clock() / 1000); }
unsigned int plat_get_ticks_us(void) { return (unsigned int)clock(); }
void plat_wait_till_us(unsigned int us_to) {
    unsigned int ticks = (unsigned int)clock();
    if (us_to > ticks) usleep(us_to - ticks);
}
void plat_sleep_ms(int ms) { usleep((unsigned int)ms * 1000); }
int plat_wait_event(int *fds_hnds, int count, int timeout_ms) { return 0; }

void *plat_mmap(unsigned long addr, size_t size, int need_exec, int is_fixed) { return malloc(size); }
void *plat_mremap(void *ptr, size_t oldsize, size_t newsize) { return realloc(ptr, newsize); }
void plat_munmap(void *ptr, size_t size) { free(ptr); }
void *plat_mem_get_for_drc(size_t size) { return NULL; }
int plat_mem_set_exec(void *ptr, size_t size) { return 0; }

int _flush_cache (char *addr, const int size, const int op) { 
    FlushCache(0); FlushCache(2); return 0;
}

int posix_memalign(void **p, size_t align, size_t size) {
    void *ptr = memalign(align, size);
    if (!ptr) return ENOMEM;
    *p = ptr;
    return 0;
}

void lprintf(const char *fmt, ...) {
    va_list vl; va_start(vl, fmt);
    vprintf(fmt, vl); va_end(vl);
}

void plat_debug_cat(char *str) {}
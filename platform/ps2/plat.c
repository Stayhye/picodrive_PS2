/*
 * PicoDrive platform interface for PS2
 * (Non-blocking, Non-stuttering BGM Engine)
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

static void bgm_thread_func(void *arg) {
    /* Initialize audsrv asynchronously inside the thread so boot isn't delayed */
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

    FILE *f = fopen("menu.adp", "rb");
    if (!f) f = fopen("MENU.ADP", "rb");
    if (!f) f = fopen("cdfs:/SKIN/MENU.ADP;1", "rb");
    if (!f) f = fopen("mc0:/PICO/MENU.ADP", "rb");

    if (!f) {
        bgm_tid = -1;
        bgm_running = 0;
        ExitDeleteThread();
        return;
    }

    struct audsrv_fmt_t adp_fmt;
    adp_fmt.bits = 16;
    adp_fmt.freq = 44100;
    adp_fmt.channels = 2;
    audsrv_set_format(&adp_fmt);

    /* 4KB buffer for smooth streaming without underruns */
    static char audio_buf[4096]; 

    while (bgm_running) {
        /* Check queued audio room to prevent blocking calls that stutter */
        if (audsrv_queued() > 16384) {
            usleep(5000); // Sleep briefly if ring buffer is full
            continue;
        }

        int bytes_read = (int)fread(audio_buf, 1, sizeof(audio_buf), f);
        if (bytes_read <= 0) {
            fseek(f, 0, SEEK_SET);
            continue;
        }

        if (!bgm_running) break;

        audsrv_play_audio(audio_buf, bytes_read);
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
    thread.initial_priority = 48; // High priority thread prevents stuttering
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
    
    /* audsrv_init() moved to background thread to eliminate 2-minute boot lag */
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
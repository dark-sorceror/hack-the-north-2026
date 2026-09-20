/* camtap: a RealSense D435(i)'s colour and depth, from the QNX Sensor
 * Framework into shared memory, for detector.py.
 *
 * The camera API (camera/camera_api.h) comes from `apk add qnx-sf-base-dev`,
 * the library from the image (libcamapi). The sensor service must be running
 * a config with the camera's units (sensor_d435i.conf; see README.md).
 *
 * Each viewfinder callback copies its frame into /dev/shmem/camtap under a
 * seqlock (seq is odd while a copy is in progress), so the reader always gets
 * the newest whole frame and never waits on the camera:
 *
 *   offset 0         camtap_shm_t header (one page)
 *   COLOR_OFF        colour, YCbYCr (YUYV), width*2 bytes per row, no padding
 *   DEPTH_OFF        depth, uint16 millimetres, width*2 bytes per row
 *
 * Build on the Pi: make        Run: ./camtap [-c colour_unit] [-d depth_unit] [-l]
 */
#include <camera/camera_api.h>
#include <fcntl.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define SHM_NAME   "/camtap"
#define MAGIC      0x43414d54u          /* "CAMT" */
#define COLOR_MAX  (1920u * 1080u * 2u)
#define DEPTH_MAX  (1280u * 720u * 2u)
#define COLOR_OFF  4096u
#define DEPTH_OFF  (COLOR_OFF + COLOR_MAX)
#define SHM_SIZE   (DEPTH_OFF + DEPTH_MAX)

/* Mirrored by detector.py's CamTap. Keep the two in step. */
typedef struct {
    _Atomic uint32_t seq;               /* odd while being written */
    uint32_t width, height, frames;
    double mono_s;                      /* CLOCK_MONOTONIC when the copy finished */
} camtap_slot_t;

typedef struct {
    uint32_t magic, version;
    camtap_slot_t color, depth;
} camtap_shm_t;

static volatile sig_atomic_t quit;
static void on_signal(int sig) { (void)sig; quit = 1; }

static uint8_t *base;
static camtap_shm_t *shm;

static double mono_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void publish(camtap_slot_t *slot, uint8_t *dst, size_t max,
                    const uint8_t *src, uint32_t w, uint32_t h, uint32_t stride) {
    size_t row = (size_t)w * 2u;        /* both formats are 2 bytes per pixel */
    if (row * h > max || stride < row) return;
    atomic_fetch_add_explicit(&slot->seq, 1, memory_order_acq_rel);    /* odd */
    for (uint32_t y = 0; y < h; y++) memcpy(dst + y * row, src + (size_t)y * stride, row);
    slot->width = w;
    slot->height = h;
    slot->frames++;
    slot->mono_s = mono_s();
    atomic_fetch_add_explicit(&slot->seq, 1, memory_order_acq_rel);    /* even */
}

static void on_frame(camera_handle_t handle, camera_buffer_t *buf, void *arg) {
    (void)handle; (void)arg;
    static _Atomic int warned;
    switch (buf->frametype) {
    case CAMERA_FRAMETYPE_YCBYCR:
        publish(&shm->color, base + COLOR_OFF, COLOR_MAX, buf->framebuf,
                buf->framedesc.ycbycr.width, buf->framedesc.ycbycr.height,
                buf->framedesc.ycbycr.stride);
        break;
    case CAMERA_FRAMETYPE_Z16:
        publish(&shm->depth, base + DEPTH_OFF, DEPTH_MAX, buf->framebuf,
                buf->framedesc.z16.width, buf->framedesc.z16.height,
                buf->framedesc.z16.stride);
        break;
    default:
        if (!atomic_exchange(&warned, 1))
            fprintf(stderr, "camtap: ignoring frame type %d (want YCBYCR %d or Z16 %d)\n",
                    (int)buf->frametype, CAMERA_FRAMETYPE_YCBYCR, CAMERA_FRAMETYPE_Z16);
    }
}

static int list_units(void) {
    unsigned n = 0;
    if (camera_get_supported_cameras(0, &n, NULL) != CAMERA_EOK) {
        fprintf(stderr, "camtap: cannot list cameras (is the sensor service running?)\n");
        return 1;
    }
    camera_unit_t units[16];
    if (n > 16) n = 16;
    camera_get_supported_cameras(n, &n, units);
    printf("camera units:");
    for (unsigned i = 0; i < n; i++) printf(" %d", (int)units[i]);
    printf("\n");
    return 0;
}

static camera_handle_t start(int unit, const char *what) {
    camera_handle_t h = CAMERA_HANDLE_INVALID;
    camera_error_t err = camera_open((camera_unit_t)unit, CAMERA_MODE_RO, &h);
    if (err != CAMERA_EOK || h == CAMERA_HANDLE_INVALID) {
        fprintf(stderr, "camtap: open %s unit %d: error %d\n", what, unit, (int)err);
        return CAMERA_HANDLE_INVALID;
    }
    err = camera_start_viewfinder(h, on_frame, NULL, NULL);
    if (err != CAMERA_EOK) {
        fprintf(stderr, "camtap: start %s unit %d: error %d\n", what, unit, (int)err);
        camera_close(h);
        return CAMERA_HANDLE_INVALID;
    }
    return h;
}

int main(int argc, char **argv) {
    int color_unit = 2, depth_unit = 1;
    for (int c; (c = getopt(argc, argv, "c:d:lh")) != -1;) {
        switch (c) {
        case 'c': color_unit = atoi(optarg); break;
        case 'd': depth_unit = atoi(optarg); break;
        case 'l': return list_units();
        default:
            fprintf(stderr, "usage: %s [-c colour_unit=2] [-d depth_unit=1] [-l]  (0 = off)\n", argv[0]);
            return 2;
        }
    }
    setvbuf(stdout, NULL, _IOLBF, 0);

    int fd = shm_open(SHM_NAME, O_RDWR | O_CREAT, 0664);
    if (fd == -1 || ftruncate(fd, SHM_SIZE) == -1) { perror("camtap: shm"); return 1; }
    base = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) { perror("camtap: mmap"); return 1; }
    close(fd);
    shm = (camtap_shm_t *)base;
    memset(shm, 0, sizeof *shm);
    shm->version = 1;
    shm->magic = MAGIC;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    camera_handle_t hc = CAMERA_HANDLE_INVALID, hd = CAMERA_HANDLE_INVALID;
    if (color_unit > 0 && (hc = start(color_unit, "colour")) == CAMERA_HANDLE_INVALID) return 1;
    if (depth_unit > 0 && (hd = start(depth_unit, "depth")) == CAMERA_HANDLE_INVALID) return 1;
    printf("camtap: colour unit %d, depth unit %d -> /dev/shmem%s\n", color_unit, depth_unit, SHM_NAME);

    uint32_t fc = 0, fdp = 0;
    while (!quit) {
        sleep(2);
        uint32_t c = shm->color.frames, d = shm->depth.frames;
        printf("camtap: colour %ux%u %.1f fps, depth %ux%u %.1f fps\n",
               shm->color.width, shm->color.height, (c - fc) / 2.0,
               shm->depth.width, shm->depth.height, (d - fdp) / 2.0);
        fc = c; fdp = d;
    }

    if (hc != CAMERA_HANDLE_INVALID) { camera_stop_viewfinder(hc); camera_close(hc); }
    if (hd != CAMERA_HANDLE_INVALID) { camera_stop_viewfinder(hd); camera_close(hd); }
    shm_unlink(SHM_NAME);
    printf("camtap: stopped\n");
    return 0;
}

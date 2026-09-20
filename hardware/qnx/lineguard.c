/* lineguard: the QNX supervisor's fail-safe output.
 *
 * Toggles one GPIO as a square wave (default GPIO17, 25 ms half-period, so
 * 20 Hz) while, and only while, the detector keeps saying SAFE. The pin
 * drives an optocoupler LED; the robot Pi's bridge sees the square wave on
 * its own GPIO17 and treats a constant level of either polarity as STOP
 * (bridge/gpio.py HeartbeatInput). So every failure here stops the robot:
 *
 *   detector says STOP           -> we hold the LED off
 *   detector silent > deadline   -> we hold the LED off  (crash, hang, stall)
 *   lineguard crashes or hangs   -> the pin freezes at its last level
 *   QNX Pi reboots or loses power-> the pin floats to its pull-down, LED off
 *   wire cut                     -> the robot's pull-up holds HIGH
 *
 * A steady "LOW means OK" level cannot give the third line: a GPIO output
 * keeps its last value after the process that set it is gone.
 *
 * Verdicts arrive as datagrams on a UNIX socket (default /tmp/lineguard.sock):
 *   "SAFE <seq> <capture_monotonic_s>"
 *   "STOP <capture_monotonic_s> <reason...>"
 * One tick handles every queued datagram, so the newest verdict wins.
 *
 * Stop at once, resume slowly: after a STOP, toggling resumes only once SAFE
 * verdicts have arrived for the whole hold time (-H, default 1000 ms). A
 * detector that flickers at its threshold, or loses a person who has come
 * close enough to fill the frame, then cannot restart the robot between two
 * frames. The hold lives here, not in the detector, because this is the part
 * that has to be right.
 *
 * Runs SCHED_FIFO above the detector, so the AI can use every core without
 * delaying a tick. -v prints wake-up lateness, the evidence for that claim.
 *
 * Build on the Pi: make    (clang, no SDP needed)
 * Run:             ./lineguard [-g 17] [-p 25] [-d 200] [-H 1000] [-P 50] [-s path] [-v]
 */
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/iomgr.h>
#include <sys/iomsg.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

/* From gitlab.com/qnx/projects/rpi-gpio resmgr/public/sys/rpi_gpio.h, which
 * the image does not install. The resource manager rejects messages without
 * this mgrid (EBADMSG). */
#define RPI_GPIO_IOMGR (_IOMGR_PRIVATE_BASE + 35)
enum { RPI_GPIO_SET_SELECT, RPI_GPIO_GET_SELECT, RPI_GPIO_WRITE, RPI_GPIO_READ };
enum { RPI_GPIO_FUNC_IN = 0, RPI_GPIO_FUNC_OUT = 1 };
typedef struct { struct _io_msg hdr; unsigned gpio; unsigned value; } rpi_gpio_msg_t;

static volatile sig_atomic_t quit;
static void on_signal(int sig) { (void)sig; quit = 1; }

static int gpio_fd = -1;
static unsigned gpio_pin = 17;

static int gpio_send(unsigned subtype, unsigned value) {
    rpi_gpio_msg_t m = { .hdr.type = _IO_MSG, .hdr.mgrid = RPI_GPIO_IOMGR,
                         .hdr.subtype = subtype, .gpio = gpio_pin, .value = value };
    return MsgSend(gpio_fd, &m, sizeof m, &m, sizeof m) == -1 ? -1 : 0;
}

static double mono_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void stamp(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);
    printf("%02d:%02d:%02d.%03ld ", tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000);
}

static void usage(const char *argv0) {
    fprintf(stderr,
            "usage: %s [-g gpio=17] [-p half_period_ms=25] [-d deadline_ms=200]\n"
            "          [-H hold_ms=1000] [-P fifo_priority=50] [-s socket=/tmp/lineguard.sock] [-v]\n", argv0);
}

int main(int argc, char **argv) {
    int half_ms = 25, deadline_ms = 200, hold_ms = 1000, prio = 50, verbose = 0;
    const char *sock_path = "/tmp/lineguard.sock";
    for (int c; (c = getopt(argc, argv, "g:p:d:H:P:s:vh")) != -1;) {
        switch (c) {
        case 'g': gpio_pin = (unsigned)atoi(optarg); break;
        case 'p': half_ms = atoi(optarg); break;
        case 'd': deadline_ms = atoi(optarg); break;
        case 'H': hold_ms = atoi(optarg); break;
        case 'P': prio = atoi(optarg); break;
        case 's': sock_path = optarg; break;
        case 'v': verbose = 1; break;
        default: usage(argv[0]); return 2;
        }
    }
    if (half_ms < 1 || deadline_ms < half_ms || hold_ms < 0) { usage(argv[0]); return 2; }
    setvbuf(stdout, NULL, _IOLBF, 0);

    struct sched_param sp = { .sched_priority = prio };
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) fprintf(stderr, "warning: SCHED_FIFO %d: %s\n", prio, strerror(rc));
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) perror("warning: mlockall");

    gpio_fd = open("/dev/gpio/msg", O_RDWR);
    if (gpio_fd == -1) { perror("open /dev/gpio/msg"); return 1; }
    if (gpio_send(RPI_GPIO_SET_SELECT, RPI_GPIO_FUNC_OUT) || gpio_send(RPI_GPIO_WRITE, 0)) {
        perror("gpio setup");
        return 1;
    }

    int sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    strncpy(addr.sun_path, sock_path, sizeof addr.sun_path - 1);
    unlink(sock_path);
    if (sock == -1 || bind(sock, (struct sockaddr *)&addr, sizeof addr) == -1) {
        perror("verdict socket");
        return 1;
    }
    fcntl(sock, F_SETFL, O_NONBLOCK);
    chmod(sock_path, 0660);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    stamp();
    printf("lineguard: GPIO%u, %d ms half-period, %d ms deadline, %d ms hold, SCHED_FIFO %d, %s: "
           "STOP until the first SAFE\n", gpio_pin, half_ms, deadline_ms, hold_ms, prio, sock_path);

    const double deadline = deadline_ms / 1e3, hold = hold_ms / 1e3;
    double last_safe = -1e9;       /* monotonic time the newest SAFE arrived */
    double hold_until = 0;         /* no toggling before this, after a STOP */
    int ok = 0, level = 0;         /* ok: currently toggling */
    char reason[160] = "no verdict yet";
    long ticks = 0;
    double max_late_us = 0, sum_late_us = 0, report_at = mono_s() + 5;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (!quit) {
        next.tv_nsec += half_ms * 1000000L;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) == EINTR && !quit) {}
        if (quit) break;
        double now = mono_s();
        double late_us = (now - (next.tv_sec + next.tv_nsec / 1e9)) * 1e6;
        if (late_us > max_late_us) max_late_us = late_us;
        sum_late_us += late_us;
        ticks++;

        /* Newest verdict wins. A STOP also forgets the last SAFE, so only a
         * SAFE sent after it can resume toggling. */
        char buf[256];
        ssize_t n;
        double stop_capture = 0;
        while ((n = recv(sock, buf, sizeof buf - 1, 0)) > 0) {
            buf[n] = '\0';
            if (strncmp(buf, "SAFE", 4) == 0) {
                last_safe = now;
            } else if (strncmp(buf, "STOP", 4) == 0) {
                last_safe = -1e9;
                hold_until = now + hold;
                char *p = buf + 4;
                stop_capture = strtod(p, &p);
                while (*p == ' ') p++;
                snprintf(reason, sizeof reason, "detector: %s", *p ? p : "STOP");
            }
        }

        int want_ok = now - last_safe <= deadline && now >= hold_until;
        if (!want_ok && ok && last_safe > 0 && now >= hold_until)
            snprintf(reason, sizeof reason, "no SAFE for %.0f ms (deadline %d ms)",
                     (now - last_safe) * 1e3, deadline_ms);

        if (want_ok) {
            level = !level;
        } else {
            level = 0;             /* LED off: the robot sees a constant level */
        }
        if (gpio_send(RPI_GPIO_WRITE, (unsigned)level) == -1) {
            perror("gpio write");  /* toggling stops either way; exit loudly */
            break;
        }

        if (want_ok != ok) {
            stamp();
            if (want_ok) {
                printf("OK    toggling\n");
            } else if (stop_capture > 0) {
                printf("STOP  %s  (%.1f ms after frame capture)\n", reason, (now - stop_capture) * 1e3);
            } else {
                printf("STOP  %s\n", reason);
            }
            ok = want_ok;
        }
        if (verbose && now >= report_at) {
            stamp();
            printf("ticks=%ld  wake-up lateness: mean %.0f us, max %.0f us  state=%s\n",
                   ticks, sum_late_us / ticks, max_late_us, ok ? "OK" : "STOP");
            report_at = now + 5;
        }
    }

    gpio_send(RPI_GPIO_WRITE, 0);
    gpio_send(RPI_GPIO_SET_SELECT, RPI_GPIO_FUNC_IN);
    close(sock);
    unlink(sock_path);
    stamp();
    printf("lineguard: stopped, GPIO%u released (LED off)\n", gpio_pin);
    return 0;
}

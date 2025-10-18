/* Antenna Tuner Controller
 * Controls a stepper motor via an ESP32 (TCP) and measures SWR using a NanoVNA.
 * Automatically optimizes tuner position for 160m, 60m, 40m, and other bands with
 * band-specific parameters for coarse and fine tuning. Stores positions in a CSV
 * file for reuse. Uses a persistent TCP connection to reduce overhead.
 *
 * Usage: ./tuner <frequency_in_MHz> (e.g., ./tuner 1.885)
 * Compile: gcc -o tuner tuner.c -lpthread -lm
 * License: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <math.h>
#include <sys/select.h>
#include <errno.h>
#include <ctype.h>
#include <sys/stat.h>

/* Configuration Constants */
#define SERIAL_PORT "/dev/ttyACM0"          // NanoVNA serial port
#define BAUDRATE B9600                      // Serial baud rate for NanoVNA
#define ESP32_IP "192.168.1.29"             // ESP32 IP address
#define ESP32_PORT 75                       // ESP32 TCP port
#define MEAS_DELAY 500                      // Delay after frequency set (ms)
#define SWR_TARGET 1.8                      // Target SWR for most bands
#define SWR_TARGET_160M 3.0                 // Relaxed SWR target for 160m
#define MIN_FREQ 1.5                        // Minimum frequency (MHz)
#define MAX_FREQ 7.5                        // Maximum frequency (MHz)
#define ESP32_TIMEOUT 360000000             // Timeout for ESP32 response (us)
#define NANOVNA_TIMEOUT 1000000             // Timeout for NanoVNA response (us)
#define CSV_FILE "tuner_positions.csv"      // File for storing positions

#define GROB_PROZENT_DEFAULT 0.75           // Default coarse movement (75%)
#define GROB_PROZENT_40M 0.50               // Coarse movement for 40m (50%)
#define GROB_PROZENT_60M 0.15               // Coarse movement for 60m (15%)
#define GROB_PROZENT_160M 0.90              // Coarse movement for 160m (90%)
#define MAX_SCAN_STEPS 2000                 // Max steps in fine-tuning scan
#define NO_IMPROVEMENT_LIMIT 200            // Max steps without SWR improvement
#define NO_IMPROVEMENT_LIMIT_160M 50        // Shorter limit for 160m (faster stop)
#define SCAN_STEP_SIZE_40M 5                // Fine-tuning step size for 40m
#define SCAN_STEP_SIZE_60M 5                // Fine-tuning step size for 60m
#define SCAN_STEP_SIZE_160M 2               // Fine-tuning step size for 160m (finer)
#define SCAN_STEP_SIZE_OTHER 10             // Fine-tuning step size for other bands

/* Structure for frequency-to-position mapping */
typedef struct { double freq; long position; } Point;

/* Calibration table for interpolation */
Point freq_points[] = {
    {1.8, 0000}, {1.85, 920}, {1.9, 1820}, {2.0, 3400},
    {3.5, 13320}, {3.65, 13710}, {3.6855, 13790}, {4.0, 14440},
    {5.35, 16080}, {5.45, 16150},
    {7.0, 16970}, {7.065, 17020}, {7.1, 17019}, {7.125, 17030}, {7.2, 17055}, {7.3, 17085}
};

/* Global Variables */
int serial_fd = -1;              // File descriptor for NanoVNA serial port
double previous_mhz = 1.8;       // Last used frequency (MHz)
long motor_position = 0;         // Current motor position (steps)
float global_best_swr = 99.0;    // Best SWR during scan
long global_best_pos = 0;        // Position with best SWR
int esp32_sock = -1;             // Persistent TCP socket for ESP32

/* Loads last known position from CSV file for initialization.
 * If no file exists, defaults to 1.8 MHz at position 0.
 */
void load_last_position() {
    FILE *file = fopen(CSV_FILE, "r");
    if (!file) {
        printf("INFO: No CSV found – starting at 1.8 MHz, Pos 0\n");
        return;
    }
    
    char line[256];
    double last_freq = 1.8;
    long last_pos = 0;
    float last_swr = 0.0;
    if (fgets(line, sizeof(line), file)) {
        while (fgets(line, sizeof(line), file)) {
            if (sscanf(line, "%lf,%ld,%f", &last_freq, &last_pos, &last_swr) >= 2) {
                previous_mhz = last_freq;
                motor_position = last_pos;
            }
        }
    }
    fclose(file);
    printf("INFO: Loaded position from CSV: %.4f MHz at Pos %ld (SWR %.2f)\n", previous_mhz, motor_position, last_swr);
}

/* Saves frequency, position, and SWR to CSV file.
 * Appends new entries and adds header if file is empty.
 */
void save_position(double freq, long pos, float swr) {
    FILE *file = fopen(CSV_FILE, "a");
    if (file) {
        struct stat st;
        if (stat(CSV_FILE, &st) != 0 || st.st_size == 0) {
            fprintf(file, "freq,position,swr\n");
        }
        fprintf(file, "%.4f,%ld,%.2f\n", freq, pos, swr);
        fclose(file);
        printf("INFO: Saved position to CSV: %.4f MHz at Pos %ld with SWR %.2f\n", freq, pos, swr);
    }
}

/* Opens serial connection to NanoVNA.
 * Returns 0 on success, -1 on failure.
 */
int open_serial() {
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) { perror("Serial-Port Error"); return -1; }
    struct termios tty;
    tcgetattr(serial_fd, &tty);
    tty.c_cflag = (CLOCAL | CREAD | CS8);
    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);
    cfmakeraw(&tty);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) { perror("tcsetattr Error"); return -1; }
    tcflush(serial_fd, TCIOFLUSH);
    return 0;
}

/* Sends a command to NanoVNA via serial port.
 * Flushes buffer and adds newline.
 */
void send_serial_cmd(const char* cmd) {
    if (serial_fd < 0) return;
    tcflush(serial_fd, TCIOFLUSH);
    printf("DEBUG: Sending Serial: %s\n", cmd);
    write(serial_fd, cmd, strlen(cmd));
    write(serial_fd, "\r\n", 2);
    usleep(100000);
}

/* Checks if NanoVNA is responsive by sending "help" command.
 * Returns 0 if "ch> " prompt received, -1 otherwise.
 */
int check_nanovna() {
    if (serial_fd < 0) return -1;
    tcflush(serial_fd, TCIOFLUSH);
    send_serial_cmd("help");
    int flags = fcntl(serial_fd, F_GETFL, 0);
    fcntl(serial_fd, F_SETFL, flags & ~O_NONBLOCK);
    char buf[1024] = {0};
    int total_read = 0;
    struct timeval tv;
    fd_set readfds;
    long total_wait = 0;
    while (total_wait < 5000000) {
        FD_ZERO(&readfds);
        FD_SET(serial_fd, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = NANOVNA_TIMEOUT;
        int ready = select(serial_fd + 1, &readfds, NULL, NULL, &tv);
        if (ready > 0) {
            int n = read(serial_fd, buf + total_read, sizeof(buf) - total_read - 1);
            if (n > 0) {
                total_read += n;
                buf[total_read] = '\0';
                if (strstr(buf, "ch> ")) {
                    fcntl(serial_fd, F_SETFL, flags);
                    tcflush(serial_fd, TCIOFLUSH);
                    return 0;
                }
            } else if (n < 0 && errno != EAGAIN) break;
        }
        usleep(200000);
        total_wait += 200000;
    }
    fcntl(serial_fd, F_SETFL, flags);
    tcflush(serial_fd, TCIOFLUSH);
    return -1;
}

/* Reads SWR from NanoVNA.
 * Returns measured SWR or -1.0 on failure.
 * Attempts reconnect on error.
 */
float read_swr() {
    usleep(500000);  // Wait for motor to settle
    if (check_nanovna() < 0) {
        printf("ERROR: NanoVNA not reachable – attempting reconnect...\n");
        close(serial_fd);
        usleep(1000000);
        if (open_serial() < 0) return -1.0;
        if (check_nanovna() < 0) return -1.0;
    }
    tcflush(serial_fd, TCIOFLUSH);
    send_serial_cmd("pause");
    usleep(300000);
    send_serial_cmd("data 0");
    usleep(300000);
    int flags = fcntl(serial_fd, F_GETFL, 0);
    fcntl(serial_fd, F_SETFL, flags & ~O_NONBLOCK);
    char buf[1024] = {0};
    int total_read = 0;
    struct timeval tv;
    fd_set readfds;
    long total_wait = 0;
    float swr = -1.0;
    while (total_wait < 5000000) {
        FD_ZERO(&readfds);
        FD_SET(serial_fd, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = NANOVNA_TIMEOUT;
        int ready = select(serial_fd + 1, &readfds, NULL, NULL, &tv);
        if (ready > 0) {
            int n = read(serial_fd, buf + total_read, sizeof(buf) - total_read - 1);
            if (n > 0) {
                total_read += n;
                buf[total_read] = '\0';
                float real, imag;
                char* line = strtok(buf, "\r\n");
                while (line != NULL) {
                    if (strncmp(line, "data", 4) == 0 || strncmp(line, "ch>", 3) == 0 || strncmp(line, "pause", 5) == 0 || line[0] == '\0') {
                        line = strtok(NULL, "\r\n");
                        continue;
                    }
                    if (sscanf(line, "%f %f", &real, &imag) == 2) {
                        float mag = sqrt(real * real + imag * imag);
                        if (mag > 0.9999) mag = 0.9999;
                        if (mag > 0.0) {
                            swr = (1.0 + mag) / (1.0 - mag + 1e-6);
                            printf("DEBUG: Calculated SWR: %.2f\n", swr);
                            fcntl(serial_fd, F_SETFL, flags);
                            send_serial_cmd("resume");
                            return swr;
                        }
                    }
                    line = strtok(NULL, "\r\n");
                }
            } else if (n < 0 && errno != EAGAIN) break;
        }
        usleep(200000);
        total_wait += 200000;
    }
    fcntl(serial_fd, F_SETFL, flags);
    send_serial_cmd("resume");
    return swr;
}

/* Sets NanoVNA frequency for SWR measurement.
 * Configures sweep and trace for accurate reading.
 */
void set_frequency(double mhz) {
    if (check_nanovna() < 0) return;
    long long hz = (long long)(mhz * 1000000.0);
    char cmd[64];
    printf("DEBUG: Setting Frequency: %.4f MHz\n", mhz);
    send_serial_cmd("pause");
    snprintf(cmd, sizeof(cmd), "freq %lld", hz);
    send_serial_cmd(cmd);
    send_serial_cmd("trace 0 SWR CH0");
    snprintf(cmd, sizeof(cmd), "sweep %lld %lld 1", hz, hz);
    send_serial_cmd(cmd);
    usleep(MEAS_DELAY * 1000);
    send_serial_cmd("resume");
}

/* Establishes persistent TCP connection to ESP32.
 * Returns 0 if connected, -1 on failure.
 */
int connect_esp32() {
    if (esp32_sock >= 0) return 0;  // Already connected
    esp32_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (esp32_sock < 0) return -1;
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(ESP32_PORT) };
    inet_pton(AF_INET, ESP32_IP, &addr.sin_addr);
    if (connect(esp32_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(esp32_sock);
        esp32_sock = -1;
        return -1;
    }
    printf("INFO: Connected to ESP32\n");
    return 0;
}

/* Closes the ESP32 TCP connection. */
void disconnect_esp32() {
    if (esp32_sock >= 0) {
        close(esp32_sock);
        esp32_sock = -1;
        printf("INFO: Disconnected from ESP32\n");
    }
}

/* Sends motor command to ESP32 (e.g., "hoch,100").
 * Returns 0 on success ("fertig" response), -1 on failure.
 */
int send_motor_cmd(const char* direction, int steps, int rounds) {
    if (steps == 0) return 0;
    if (connect_esp32() < 0) return -1;
    
    printf("DEBUG: Sending Motor Command: %s, steps=%d, rounds=%d\n", direction, steps, rounds);
    char buf[32];
    
    snprintf(buf, sizeof(buf), "rounds,%d\n", rounds);
    write(esp32_sock, buf, strlen(buf));
    usleep(300000);
    
    snprintf(buf, sizeof(buf), "%s,%d\n", direction, steps);
    write(esp32_sock, buf, strlen(buf));
    usleep(300000);
    
    char full_response[1024] = {0};
    int total_read = 0;
    long total_wait = 0;
    int fertig_found = 0;
    while (total_wait < ESP32_TIMEOUT && !fertig_found) {
        struct timeval tv = {0, 1000000};
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(esp32_sock, &readfds);
        int ready = select(esp32_sock + 1, &readfds, NULL, NULL, &tv);
        if (ready > 0) {
            int n = read(esp32_sock, full_response + total_read, sizeof(full_response) - total_read - 1);
            if (n > 0) {
                total_read += n;
                full_response[total_read] = '\0';
                printf("DEBUG: ESP32 Response: %s\n", full_response + total_read - n);
                
                char temp[1024];
                strncpy(temp, full_response, sizeof(temp) - 1);
                temp[sizeof(temp) - 1] = '\0';
                char *p = temp;
                while (*p) {
                    *p = tolower((unsigned char)*p);
                    p++;
                }
                if (strstr(temp, "fertig")) {
                    fertig_found = 1;
                }
            }
        }
        total_wait += 1000000;
    }
    return fertig_found ? 0 : -1;
}

/* Sets motor speed on ESP32 (e.g., "mittel").
 * Waits for "setze speed" response.
 */
void set_motor_speed(const char* speed) {
    if (connect_esp32() < 0) return;
    
    printf("DEBUG: Setting Motor Speed: %s\n", speed);
    char buf[32];
    snprintf(buf, sizeof(buf), "%s\n", speed);
    write(esp32_sock, buf, strlen(buf));
    usleep(300000);
    
    char response[1024] = {0};
    int total_read = 0;
    long total_wait = 0;
    int speed_set = 0;
    while (total_wait < 10000000 && !speed_set) {
        struct timeval tv = {0, 1000000};
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(esp32_sock, &readfds);
        int ready = select(esp32_sock + 1, &readfds, NULL, NULL, &tv);
        if (ready > 0) {
            int n = read(esp32_sock, response + total_read, sizeof(response) - total_read - 1);
            if (n > 0) {
                total_read += n;
                response[total_read] = '\0';
                printf("DEBUG: ESP32 Speed Response: %s\n", response + total_read - n);
                
                char temp[1024];
                strncpy(temp, response, sizeof(temp) - 1);
                temp[sizeof(temp) - 1] = '\0';
                char *p = temp;
                while (*p) {
                    *p = tolower((unsigned char)*p);
                    p++;
                }
                if (strstr(temp, "setze speed")) {
                    speed_set = 1;
                }
            }
        }
        total_wait += 1000000;
    }
}

/* Interpolates motor position for given frequency.
 * Checks CSV for known good positions (SWR < 2.5) first.
 * Falls back to linear interpolation from freq_points table.
 */
long interpolate_position(double mhz) {
    FILE *file = fopen(CSV_FILE, "r");
    if (file) {
        char line[256];
        double freq;
        long pos;
        float swr;
        if (fgets(line, sizeof(line), file)) {
            while (fgets(line, sizeof(line), file)) {
                if (sscanf(line, "%lf,%ld,%f", &freq, &pos, &swr) == 3) {
                    if (fabs(freq - mhz) < 0.001 && swr < 2.5) {
                        fclose(file);
                        printf("INFO: CSV override: %.4f MHz at Pos %ld (SWR %.2f)\n", freq, pos, swr);
                        return pos;
                    }
                }
            }
        }
        fclose(file);
    }
    
    for (int i = 0; i < sizeof(freq_points)/sizeof(Point) - 1; i++) {
        if (mhz >= freq_points[i].freq && mhz <= freq_points[i+1].freq) {
            double f1 = freq_points[i].freq, f2 = freq_points[i+1].freq;
            long p1 = freq_points[i].position, p2 = freq_points[i+1].position;
            double linear_pos = p1 + (mhz - f1) / (f2 - f1) * (p2 - p1);
            long scaled_pos = (long)linear_pos;
            printf("DEBUG: Interpolation: %.4f MHz -> Pos %ld\n", mhz, scaled_pos);
            return scaled_pos;
        }
    }
    printf("ERROR: No interpolation for %.4f MHz\n", mhz);
    return -1;
}

/* Checks if frequency is in 40m band (7.0–7.3 MHz). */
int is_40m_band(double freq) {
    return (freq >= 7.0 && freq <= 7.3);
}

/* Checks if frequency is in 60m band (5.0–5.5 MHz). */
int is_60m_band(double freq) {
    return (freq >= 5.0 && freq <= 5.5);
}

/* Checks if frequency is in 160m band (1.8–2.0 MHz). */
int is_160m_band(double freq) {
    return (freq >= 1.8 && freq <= 2.0);
}

/* Fine-tuning scan to find optimal SWR.
 * Uses band-specific step sizes (e.g., 2 for 160m) and stops when target SWR
 * is reached or no improvement occurs for a set number of steps.
 */
void scan_and_find_best(double mhz, const char* dir) {
    int cmd_steps = SCAN_STEP_SIZE_OTHER;
    int no_improvement_limit = NO_IMPROVEMENT_LIMIT;
    if (is_40m_band(mhz)) cmd_steps = SCAN_STEP_SIZE_40M;
    else if (is_60m_band(mhz)) cmd_steps = SCAN_STEP_SIZE_60M;
    else if (is_160m_band(mhz)) {
        cmd_steps = SCAN_STEP_SIZE_160M;  // Finer steps for 160m
        no_improvement_limit = NO_IMPROVEMENT_LIMIT_160M;  // Faster stop
    }

    float target_swr = is_160m_band(mhz) ? SWR_TARGET_160M : SWR_TARGET;
    printf("INFO: Target SWR for this band: %.1f\n", target_swr);
    
    float last_swr = read_swr();
    global_best_swr = last_swr;
    global_best_pos = motor_position;
    int no_improvement_count = 0;
    
    printf("INFO: Starting scan in direction %s with step size %d\n", dir, cmd_steps);
    
    for (int i = 0; i < MAX_SCAN_STEPS; i++) {
        set_motor_speed("mittel");
        if (send_motor_cmd(dir, cmd_steps, 1) < 0) break;
        motor_position += (strcmp(dir, "hoch") == 0 ? cmd_steps : -cmd_steps);
        
        float new_swr = read_swr();
        printf("INFO: Scan Step %d: SWR %.2f (Pos %ld)\n", i+1, new_swr, motor_position);
        
        if (new_swr < global_best_swr - 0.05) {
            global_best_swr = new_swr;
            global_best_pos = motor_position;
            no_improvement_count = 0;
            printf("INFO: Better! New best SWR %.2f at Pos %ld\n", global_best_swr, global_best_pos);
        } else {
            no_improvement_count++;
        }
        
        if (global_best_swr < target_swr) {
            printf("INFO: Target SWR reached – stopping\n");
            break;
        }
        
        if (no_improvement_count > no_improvement_limit) {
            printf("INFO: %d steps without improvement – stopping scan\n", no_improvement_limit);
            break;
        }
    }
    
    long delta = global_best_pos - motor_position;
    if (labs(delta) > 0) {
        const char* back_dir = (delta > 0) ? "hoch" : "tief";
        printf("INFO: Moving back to best position: %ld steps %s\n", labs(delta), back_dir);
        set_motor_speed("mittel");
        send_motor_cmd(back_dir, labs(delta), 1);
        motor_position = global_best_pos;
    }
    
    printf("INFO: Final SWR: %.2f at Pos %ld\n", global_best_swr, motor_position);
    save_position(mhz, motor_position, global_best_swr);
}

/* Moves motor to target frequency position.
 * Performs coarse movement (band-specific percentage) followed by fine-tuning.
 */
void move_to_freq(double mhz) {
    long target_pos = interpolate_position(mhz);
    if (target_pos < 0) return;
    
    long delta = target_pos - motor_position;
    const char* initial_dir = (delta > 0) ? "hoch" : "tief";
    int abs_delta = labs(delta);
    
    double grob_prozent = GROB_PROZENT_DEFAULT;
    if (is_40m_band(mhz)) grob_prozent = GROB_PROZENT_40M;
    else if (is_60m_band(mhz)) grob_prozent = GROB_PROZENT_60M;
    else if (is_160m_band(mhz)) grob_prozent = GROB_PROZENT_160M;
    
    int early_steps = (int)(abs_delta * grob_prozent);
    
    printf("INFO: Coarse movement: %d steps %s (%.0f%% of %d)\n", early_steps, initial_dir, grob_prozent*100, abs_delta);
    set_motor_speed("mittel");
    int remaining = early_steps;
    while (remaining > 0) {
        int steps = (remaining > 200) ? 200 : remaining;
        send_motor_cmd(initial_dir, steps, 1);
        remaining -= steps;
    }
    motor_position += (delta > 0 ? early_steps : -early_steps);
    
    scan_and_find_best(mhz, initial_dir);
    
    previous_mhz = mhz;
}

/* Main tuning function.
 * Sets frequency and moves to optimal position.
 * Returns 0 on success (SWR below target), -1 otherwise.
 */
int auto_tune(double mhz) {
    printf("INFO: Tuning for %.4f MHz\n", mhz);
    set_frequency(mhz);
    move_to_freq(mhz);
    return (global_best_swr < SWR_TARGET) ? 0 : -1;
}

/* Program entry point.
 * Takes frequency (MHz) as argument and performs tuning.
 */
int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <freq_in_MHz>\n", argv[0]);
        return 1;
    }
    double mhz = atof(argv[1]);
    if (mhz < MIN_FREQ || mhz > MAX_FREQ) {
        fprintf(stderr, "Frequency out of range %.1f-%.1f MHz\n", MIN_FREQ, MAX_FREQ);
        return 1;
    }
    
    load_last_position();
    
    if (open_serial() < 0) return 1;
    if (check_nanovna() < 0) {
        close(serial_fd);
        return 1;
    }
    int status = auto_tune(mhz);
    close(serial_fd);
    disconnect_esp32();
    printf("Tuning %s\n", status == 0 ? "successful" : "failed");
    return status;
}
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

#define SERIAL_PORT "/dev/ttyACM0"
#define BAUDRATE B9600
#define ESP32_IP "192.168.1.29"
#define ESP32_PORT 75
#define MEAS_DELAY 500
#define SWR_TARGET 1.8
#define SWR_TARGET_160M 3.0
#define MIN_FREQ 1.5
#define MAX_FREQ 7.5
#define ESP32_TIMEOUT 360000000
#define NANOVNA_TIMEOUT 1000000
#define CSV_FILE "tuner_positions.csv"

#define GROB_PROZENT_DEFAULT 1
#define GROB_PROZENT_40M 0.60
#define GROB_PROZENT_60M 0.55
#define GROB_PROZENT_160M 0.95
#define MAX_SCAN_STEPS 2000
#define NO_IMPROVEMENT_LIMIT 600
#define SCAN_STEP_SIZE_40M 5
#define SCAN_STEP_SIZE_60M 5
#define SCAN_STEP_SIZE_160M 20  // Gröbere Schritte für groben Scan im 160m
#define SCAN_STEP_SIZE_OTHER 20
#define FINE_SCAN_STEP_SIZE_160M 1  // Feinere Schritte für Feintuning im 160m

typedef struct { double freq; long position; } Point;
Point freq_points[] = {
    {1.8, 0000}, {1.85, 920}, {1.9, 1820}, {2.0, 3400},
    {3.5, 13320}, {3.65, 13710}, {3.6855, 13790}, {4.0, 14440},
    {5.35, 16080}, {5.45, 16150},
    {7.0, 16970}, {7.065, 17020}, {7.1, 17019}, {7.125, 17030}, {7.2, 17055}, {7.3, 17085}
};

int serial_fd = -1;
double previous_mhz = 1.8;
long motor_position = 0;
float global_best_swr = 99.0;
long global_best_pos = 0;
int esp32_sock = -1;

void load_last_position() {
    FILE *file = fopen(CSV_FILE, "r");
    if (!file) {
        printf("INFO: Keine CSV gefunden – starte bei 1.8 MHz, Pos 0\n");
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
    printf("INFO: Geladene Position aus CSV: %.4f MHz bei Pos %ld (SWR %.2f)\n", previous_mhz, motor_position, last_swr);
}

void save_position(double freq, long pos, float swr) {
    FILE *file = fopen(CSV_FILE, "a");
    if (file) {
        struct stat st;
        if (stat(CSV_FILE, &st) != 0 || st.st_size == 0) {
            fprintf(file, "freq,position,swr\n");
        }
        fprintf(file, "%.4f,%ld,%.2f\n", freq, pos, swr);
        fclose(file);
        printf("INFO: Position gespeichert in CSV: %.4f MHz bei Pos %ld mit SWR %.2f\n", freq, pos, swr);
    }
}

int open_serial() {
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) { perror("Serial-Port Fehler"); return -1; }
    struct termios tty;
    tcgetattr(serial_fd, &tty);
    tty.c_cflag = (CLOCAL | CREAD | CS8);
    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);
    cfmakeraw(&tty);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) { perror("tcsetattr Fehler"); return -1; }
    tcflush(serial_fd, TCIOFLUSH);
    return 0;
}

void send_serial_cmd(const char* cmd) {
    if (serial_fd < 0) return;
    tcflush(serial_fd, TCIOFLUSH);
    printf("DEBUG: Sende Serial: %s\n", cmd);
    write(serial_fd, cmd, strlen(cmd));
    write(serial_fd, "\r\n", 2);
    usleep(100000);
}

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

float read_swr() {
    usleep(500000);
    if (check_nanovna() < 0) {
        printf("ERROR: NanoVNA nicht erreichbar – reconnect...\n");
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
                            printf("DEBUG: Berechnetes SWR: %.2f\n", swr);
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

void set_frequency(double mhz) {
    if (check_nanovna() < 0) return;
    long long hz = (long long)(mhz * 1000000.0);
    char cmd[64];
    printf("DEBUG: Setze Frequenz: %.4f MHz\n", mhz);
    send_serial_cmd("pause");
    snprintf(cmd, sizeof(cmd), "freq %lld", hz);
    send_serial_cmd(cmd);
    send_serial_cmd("trace 0 SWR CH0");
    snprintf(cmd, sizeof(cmd), "sweep %lld %lld 1", hz, hz);
    send_serial_cmd(cmd);
    usleep(MEAS_DELAY * 1000);
    send_serial_cmd("resume");
}

int connect_esp32() {
    if (esp32_sock >= 0) return 0;
    esp32_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (esp32_sock < 0) return -1;
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(ESP32_PORT) };
    inet_pton(AF_INET, ESP32_IP, &addr.sin_addr);
    if (connect(esp32_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(esp32_sock);
        esp32_sock = -1;
        return -1;
    }
    printf("INFO: Verbunden mit ESP32\n");
    return 0;
}

void disconnect_esp32() {
    if (esp32_sock >= 0) {
        close(esp32_sock);
        esp32_sock = -1;
        printf("INFO: Verbindung zu ESP32 getrennt\n");
    }
}

int send_motor_cmd(const char* direction, int steps, int rounds) {
    if (steps == 0) return 0;
    if (connect_esp32() < 0) return -1;
    
    printf("DEBUG: Sende Motor-Befehl: %s, steps=%d, rounds=%d\n", direction, steps, rounds);
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
                printf("DEBUG: ESP32 Antwort: %s\n", full_response + total_read - n);
                
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

void set_motor_speed(const char* speed) {
    if (connect_esp32() < 0) return;
    
    printf("DEBUG: Setze Motor-Geschwindigkeit: %s\n", speed);
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
                printf("DEBUG: ESP32 Speed-Antwort: %s\n", response + total_read - n);
                
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
                        printf("INFO: CSV-Vorrang: %.4f MHz bei Pos %ld (SWR %.2f)\n", freq, pos, swr);
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
    printf("ERROR: Keine Interpolation für %.4f MHz\n", mhz);
    return -1;
}

int is_40m_band(double freq) {
    return (freq >= 7.0 && freq <= 7.3);
}

int is_60m_band(double freq) {
    return (freq >= 5.0 && freq <= 5.5);
}

int is_160m_band(double freq) {
    return (freq >= 1.8 && freq <= 2.0);
}

void scan_and_find_best(double mhz, const char* dir) {
    int cmd_steps = SCAN_STEP_SIZE_OTHER;
    if (is_40m_band(mhz)) cmd_steps = SCAN_STEP_SIZE_40M;
    else if (is_60m_band(mhz)) cmd_steps = SCAN_STEP_SIZE_60M;
    else if (is_160m_band(mhz)) cmd_steps = SCAN_STEP_SIZE_160M;

    float target_swr = is_160m_band(mhz) ? SWR_TARGET_160M : SWR_TARGET;
    printf("INFO: Ziel-SWR für dieses Band: %.1f\n", target_swr);
    
    float last_swr = read_swr();
    global_best_swr = last_swr;
    global_best_pos = motor_position;
    int no_improvement_count = 0;
    
    printf("INFO: Starte groben Scan in Richtung %s mit Schrittgröße %d\n", dir, cmd_steps);
    
    // Grober Scan
    for (int i = 0; i < MAX_SCAN_STEPS; i++) {
        set_motor_speed("mittel");
        if (send_motor_cmd(dir, cmd_steps, 1) < 0) break;
        motor_position += (strcmp(dir, "hoch") == 0 ? cmd_steps : -cmd_steps);
        
        float new_swr = read_swr();
        printf("INFO: Grober Scan Schritt %d: SWR %.2f (Pos %ld)\n", i+1, new_swr, motor_position);
        
        if (new_swr < global_best_swr - 0.05) {
            global_best_swr = new_swr;
            global_best_pos = motor_position;
            no_improvement_count = 0;
            printf("INFO: Besser! Neues bestes SWR %.2f bei Pos %ld\n", global_best_swr, global_best_pos);
        } else {
            no_improvement_count++;
        }
        
        if (global_best_swr < target_swr) {
            printf("INFO: Ziel-SWR erreicht – stoppe groben Scan\n");
            break;
        }
        
        if (no_improvement_count > NO_IMPROVEMENT_LIMIT / 2) {  // Früher stoppen bei Stagnation
            printf("INFO: %d Schritte ohne Verbesserung – stoppe groben Scan\n", NO_IMPROVEMENT_LIMIT / 2);
            break;
        }
    }
    
    // Feintuning nur für 160m-Band
    if (is_160m_band(mhz) && global_best_swr > 2.0) {  // Nur wenn noch Potenzial (manuell 1.86 möglich)
        const char* fine_dir = dir;  // Starte Feintuning in gleicher Richtung
        int fine_steps = FINE_SCAN_STEP_SIZE_160M;
        printf("INFO: Starte Feintuning für 160m mit Schrittgröße %d\n", fine_steps);
        
        // Rücksetze auf beste grobe Position
        long delta_back = motor_position - global_best_pos;
        if (labs(delta_back) > 0) {
            const char* back_dir = (delta_back > 0) ? "tief" : "hoch";
            send_motor_cmd(back_dir, labs(delta_back), 1);
            motor_position = global_best_pos;
        }
        
        // Feintuning: Kleinere Schritte, engerer Stopp
        no_improvement_count = 0;
        for (int i = 0; i < 400; i++) {  // Max 400 feine Schritte
            set_motor_speed("langsam");  // Langsamer für Präzision
            if (send_motor_cmd(fine_dir, fine_steps, 1) < 0) break;
            motor_position += (strcmp(fine_dir, "hoch") == 0 ? fine_steps : -fine_steps);
            
            float new_swr = read_swr();
            printf("INFO: Fein Scan Schritt %d: SWR %.2f (Pos %ld)\n", i+1, new_swr, motor_position);
            
            if (new_swr < global_best_swr - 0.02) {  // Feinere Verbesserungsschwelle
                global_best_swr = new_swr;
                global_best_pos = motor_position;
                no_improvement_count = 0;
            } else {
                no_improvement_count++;
            }
            
            if (global_best_swr < 2.0) break;  // Besser als dein manuelles 1.86 anstreben
            if (no_improvement_count > 50) break;
        }
        
        // Rückfahrt zur besten Position
        long delta = global_best_pos - motor_position;
        if (labs(delta) > 0) {
            const char* back_dir = (delta > 0) ? "hoch" : "tief";
            send_motor_cmd(back_dir, labs(delta), 1);
            motor_position = global_best_pos;
        }
    }
    
    printf("INFO: Final SWR: %.2f bei Pos %ld\n", global_best_swr, motor_position);
    save_position(mhz, motor_position, global_best_swr);
}

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
    
    printf("INFO: Grobe Bewegung: %d Schritte %s (%.0f%% von %d)\n", early_steps, initial_dir, grob_prozent*100, abs_delta);
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

int auto_tune(double mhz) {
    printf("INFO: Tuning für %.4f MHz\n", mhz);
    set_frequency(mhz);
    move_to_freq(mhz);
    return (global_best_swr < (is_160m_band(mhz) ? SWR_TARGET_160M : SWR_TARGET)) ? 0 : -1;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <freq_in_MHz>\n", argv[0]);
        return 1;
    }
    double mhz = atof(argv[1]);
    if (mhz < MIN_FREQ || mhz > MAX_FREQ) {
        fprintf(stderr, "Frequenz außerhalb %.1f-%.1f MHz\n", MIN_FREQ, MAX_FREQ);
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
    printf("Tuning %s\n", status == 0 ? "erfolgreich" : "fehlgeschlagen");
    return status;
}
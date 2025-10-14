#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <microhttpd.h>
#include <math.h>
#include <ctype.h>
#include <sys/select.h>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyACM0"
#define BAUDRATE B9600
#define ESP32_IP "192.168.1.29"
#define ESP32_PORT 75
#define HTTP_PORT 8080
#define MAX_STEPS 2000
#define STEP_SIZE 10
#define MEAS_DELAY 500
#define MEMORY_FILE "tuning_memory.csv"
#define SWR_TARGET 3.0
#define MIN_FREQ 1.5
#define MAX_FREQ 7.5
#define ESP32_TIMEOUT 360000000  // 6 Minuten in Mikrosekunden
#define NANOVNA_TIMEOUT 500000
#define STEPS_PER_UNIT 41
#define MAX_ROUNDS 100
#define BAND_CHANGE_UNITS_160_TO_80 976
#define BAND_CHANGE_UNITS_DEFAULT 15
#define SWR_GOOD_THRESHOLD 5.0

typedef struct { double freq; long position; } Point;
Point freq_points[] = {{1.8, 0}, {2.0, 3200}, {3.5, 13150}, {4.0, 14350}, {5.35, 16000}, {5.45, 16100}, {7.0, 17050}, {7.3, 17110}};
int serial_fd = -1;
char esp32_ip[16] = ESP32_IP;
double previous_mhz = 0.0;
char previous_band[10] = "";
long motor_position = 0;
char last_direction[5] = "";

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
    while (total_wait < 3000000) {
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
                printf("DEBUG: NanoVNA: %s\n", buf + total_read - n);
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
    if (check_nanovna() < 0) {
        printf("DEBUG: SWR-Messung abgebrochen: NanoVNA nicht verfügbar\n");
        return -1.0;
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
                printf("DEBUG: SWR-Daten: %s\n", buf + total_read - n);
                float real, imag;
                char* line = strtok(buf, "\r\n");
                while (line) {
                    if (strncmp(line, "data", 4) == 0 || strncmp(line, "ch>", 3) == 0 || strncmp(line, "pause", 5) == 0 || line[0] == '\0') {
                        printf("DEBUG: Ignoriere Zeile: %s\n", line);
                        line = strtok(NULL, "\r\n");
                        continue;
                    }
                    if (sscanf(line, "%f %f", &real, &imag) == 2) {
                        float mag = sqrt(real * real + imag * imag);
                        if (mag > 0.9999) mag = 0.9999;
                        if (mag > 0.0) {
                            swr = (1.0 + mag) / (1.0 - mag + 1e-6);
                            printf("DEBUG: Berechnetes SWR: %.2f (real: %.6f, imag: %.6f, mag: %.6f)\n", swr, real, imag, mag);
                            fcntl(serial_fd, F_SETFL, flags);
                            send_serial_cmd("resume");
                            return swr;
                        }
                    }
                    printf("DEBUG: Ungültige SWR-Zeile: %s\n", line);
                    line = strtok(NULL, "\r\n");
                }
            } else if (n < 0 && errno != EAGAIN) {
                printf("DEBUG: read-Fehler in read_swr\n");
                break;
            }
        } else if (ready == 0) {
            printf("DEBUG: SWR-Timeout (wait: %ld us)\n", total_wait);
        }
        usleep(200000);
        total_wait += 200000;
    }
    fcntl(serial_fd, F_SETFL, flags);
    send_serial_cmd("resume");
    printf("DEBUG: SWR-Messung fehlgeschlagen\n");
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

void set_speed(const char* speed) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return;
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(ESP32_PORT) };
    inet_pton(AF_INET, esp32_ip, &addr.sin_addr);
    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) { close(sock); return; }
    char buf[32];
    snprintf(buf, sizeof(buf), "%s\n", speed);
    printf("DEBUG: Setze Geschwindigkeit: %s\n", speed);
    write(sock, buf, strlen(buf));
    usleep(300000);
    close(sock);
}

int send_motor_cmd(const char* direction, int steps, int rounds) {
    if (steps == 0) return 0;
    printf("DEBUG: Sende Motor-Befehl: %s, steps=%d, rounds=%d (Total: %d)\n", direction, steps, rounds, steps * rounds);
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) { perror("Socket Fehler"); return -1; }
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(ESP32_PORT) };
    inet_pton(AF_INET, esp32_ip, &addr.sin_addr);
    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) { 
        printf("DEBUG: ESP32 Verbindung fehlgeschlagen\n");
        close(sock); 
        return -1; 
    }
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags & ~O_NONBLOCK);
    char buf[32], response[128] = {0}, full_response[1024] = {0};
    int total_read = 0;
    snprintf(buf, sizeof(buf), "rounds,%d\n", rounds);
    printf("DEBUG: Sende ESP32: %s", buf);
    write(sock, buf, strlen(buf));
    usleep(300000);
    snprintf(buf, sizeof(buf), "%s,%d\n", direction, steps);
    printf("DEBUG: Sende ESP32: %s", buf);
    write(sock, buf, strlen(buf));
    usleep(300000);
    
    // Warten auf "Fertig" – bis zu 6 Minuten
    long total_wait = 0;
    struct timeval tv;
    fd_set readfds;
    int fertig_found = 0;
    printf("DEBUG: Warte auf 'Fertig' vom ESP32 (max. 6 Minuten)...\n");
    while (total_wait < ESP32_TIMEOUT && !fertig_found) {
        FD_ZERO(&readfds);
        FD_SET(sock, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 1000000;  // 1 Sekunde pro Schleife
        int ready = select(sock + 1, &readfds, NULL, NULL, &tv);
        if (ready > 0) {
            int n = read(sock, response, sizeof(response) - 1);
            if (n > 0) {
                response[n] = '\0';
                if (total_read + n < sizeof(full_response) - 1) {
                    strncat(full_response, response, sizeof(full_response) - total_read - 1);
                    total_read += n;
                }
                printf("DEBUG: ESP32 Antwort: %s\n", response);
                if (strstr(full_response, "fertig") || strstr(full_response, "Fertig")) {
                    printf("DEBUG: 'Fertig' empfangen nach %ld Sekunden\n", total_wait / 1000000);
                    fertig_found = 1;
                }
            } else if (n <= 0) {
                printf("DEBUG: ESP32 Verbindung geschlossen während Wartezeit\n");
                break;
            }
        }
        total_wait += 1000000;
        if (total_wait % 10000000 == 0) {  // Alle 10 Sekunden Status
            printf("DEBUG: Warte weiter auf 'Fertig'... (%ld Sekunden vergangen)\n", total_wait / 1000000);
        }
    }
    close(sock);
    if (!fertig_found) {
        printf("DEBUG: Timeout nach 6 Minuten – kein 'Fertig' empfangen\n");
        return -1;
    }
    return 0;
}

const char* get_band(double mhz) {
    if (mhz >= 1.8 && mhz <= 2.0) return "160m";
    if (mhz >= 3.5 && mhz <= 4.0) return "80m";
    if (mhz >= 5.35 && mhz <= 5.45) return "60m";
    if (mhz >= 7.0 && mhz <= 7.3) return "40m";
    return "unknown";
}

long get_position_from_memory(double mhz) {
    FILE* file = fopen(MEMORY_FILE, "r");
    if (!file) {
        printf("DEBUG: Datei %s nicht gefunden\n", MEMORY_FILE);
        return -1;
    }
    char line[256];
    long position = -1;
    double delta = INFINITY;
    while (fgets(line, sizeof(line), file)) {
        char* token = strtok(line, ",");
        double freq = atof(token);
        strtok(NULL, ","); // band
        strtok(NULL, ","); // steps
        strtok(NULL, ","); // rounds
        token = strtok(NULL, ",\n\r");
        long tmp_position = token ? atol(token) : -1;
        double current_delta = fabs(mhz - freq);
        if (current_delta < delta && current_delta <= 0.01) {
            delta = current_delta;
            position = tmp_position;
        }
    }
    fclose(file);
    printf("DEBUG: Position für %.4f MHz aus Memory: %ld\n", mhz, position);
    return position;
}

int load_memory(double mhz, int* steps, int* rounds) {
    FILE* file = fopen(MEMORY_FILE, "r");
    if (!file) return -1;
    char line[256];
    double delta = INFINITY;
    int closest_steps = 0, closest_rounds = 0;
    long tmp_position = 0;
    while (fgets(line, sizeof(line), file)) {
        char* token = strtok(line, ",");
        double freq = atof(token);
        strtok(NULL, ","); // band
        token = strtok(NULL, ",");
        closest_steps = token ? atoi(token) : 0;
        token = strtok(NULL, ",");
        closest_rounds = token ? atoi(token) : 0;
        token = strtok(NULL, ",\n\r");
        tmp_position = token ? atol(token) : -1;
        double current_delta = fabs(mhz - freq);
        if (current_delta < delta && current_delta <= 0.01) {
            delta = current_delta;
            *steps = closest_steps;
            *rounds = closest_rounds;
        }
    }
    fclose(file);
    if (delta <= 0.01) {
        printf("DEBUG: load_memory fand Position: %ld\n", tmp_position);
        return 0;
    }
    return -1;
}

void save_memory(double mhz, int steps, int rounds) {
    FILE* file = fopen(MEMORY_FILE, "a");
    if (file) {
        fprintf(file, "%.4f,%s,%d,%d,%ld\n", mhz, get_band(mhz), steps, rounds, motor_position);
        fclose(file);
        printf("DEBUG: Position gespeichert in CSV: %.4f,%s,%d,%d,%ld\n", mhz, get_band(mhz), steps, rounds, motor_position);
    } else {
        printf("DEBUG: Fehler beim Speichern in CSV\n");
    }
}

void move_to_position(double mhz, long target_pos) {
    long delta_units = target_pos - motor_position;
    long total_steps = labs(delta_units) * STEPS_PER_UNIT;
    char direction[10] = "";
    if (delta_units > 0) strcpy(direction, "hoch");
    else if (delta_units < 0) strcpy(direction, "tief");
    else return;
    printf("DEBUG: Bewege %s um %ld Einheiten (%ld Schritte)\n", direction, labs(delta_units), total_steps);
    
    long remaining_steps = total_steps;
    while (remaining_steps > 0) {
        // Max 10000 Schritte pro Befehl (200 steps * 50 rounds)
        long steps_to_send = (remaining_steps > 10000) ? 10000 : remaining_steps;
        int steps = 200;
        int rounds = steps_to_send / 200;
        if (steps_to_send % 200 != 0) rounds++;
        if (rounds > 50) {
            rounds = 50;
            steps = 200;
        }
        if (steps_to_send <= 200) {
            steps = steps_to_send;
            rounds = 1;
        }

        printf("DEBUG: Sende Batch: steps=%d, rounds=%d (Total: %d)\n", steps, rounds, steps * rounds);
        if (send_motor_cmd(direction, steps, rounds) < 0) {
            printf("DEBUG: Motor-Befehl fehlgeschlagen – breche ab\n");
            return;
        }
        remaining_steps -= steps * rounds;
    }
    motor_position = target_pos;
}

void handle_band_change(double mhz) {
    const char* current_band = get_band(mhz);
    printf("DEBUG: Bandwechsel zu %s\n", current_band);
    long target_pos = get_position_from_memory(mhz);
    
    // Immer große Bewegung, wenn keine Position in CSV
    if (target_pos < 0) {
        // Prüfe spezifischen Wechsel von 160m zu 80m
        int units = BAND_CHANGE_UNITS_DEFAULT;
        if (strcmp(previous_band, "160m") == 0 && strcmp(current_band, "80m") == 0) {
            units = BAND_CHANGE_UNITS_160_TO_80;
            printf("DEBUG: Spezieller Bandwechsel 160m -> 80m – verwende %d Einheiten\n", units);
            set_speed("power");  // Schnellmodus für großen Sprung
        } else {
            printf("DEBUG: Normaler Bandwechsel – verwende %d Einheiten\n", units);
        }
        long new_pos = motor_position + units;
        move_to_position(mhz, new_pos);
        if (units == BAND_CHANGE_UNITS_160_TO_80) {
            set_speed("normal");  // Zurück auf normal
        }
    } else {
        // Normale Bewegung bei bekannter Position
        if (previous_mhz == 0.0) {
            FILE* file = fopen(MEMORY_FILE, "r");
            if (file) {
                char line[256];
                double last_freq = 0.0;
                long last_pos = 0;
                while (fgets(line, sizeof(line), file)) {
                    char* token = strtok(line, ",");
                    double freq = atof(token);
                    strtok(NULL, ",");
                    strtok(NULL, ",");
                    strtok(NULL, ",");
                    token = strtok(NULL, ",\n\r");
                    long tmp_pos = token ? atol(token) : -1;
                    if (freq > last_freq && tmp_pos >= 0) {
                        last_freq = freq;
                        last_pos = tmp_pos;
                    }
                }
                fclose(file);
                if (last_freq > 0.0) {
                    printf("DEBUG: Geladene Startposition aus Memory: %ld (von %.4f MHz)\n", last_pos, last_freq);
                    motor_position = last_pos;
                } else {
                    motor_position = 0;
                }
            } else {
                motor_position = 0;
            }
            move_to_position(mhz, target_pos);
        } else {
            move_to_position(mhz, target_pos);
        }
    }
    
    previous_mhz = mhz;
    strcpy(previous_band, current_band);
}

int auto_tune(double mhz) {
    printf("DEBUG: Start Tuning für %.4f MHz\n", mhz);
    handle_band_change(mhz);
    set_frequency(mhz);
    float current_swr = read_swr();
    printf("DEBUG: Initial SWR: %.2f\n", current_swr);
    if (current_swr < 0.0) {
        printf("DEBUG: Abbruch: Ungültiger SWR\n");
        previous_mhz = mhz;
        strcpy(previous_band, get_band(mhz));
        return -1;
    }
    float best_swr_ever = current_swr;
    char direction[5] = "hoch";
    int total_steps = 0;
    int cautious_mode = 0;

    while (total_steps < MAX_STEPS) {
        printf("DEBUG: Tuning-Schritt %d, Richtung: %s\n", total_steps, direction);
        
        int step_size = cautious_mode ? 1 : STEP_SIZE;
        int round_size = cautious_mode ? 1 : (STEP_SIZE / 10 + 1);
        
        if (send_motor_cmd(direction, step_size, round_size) < 0) {
            printf("DEBUG: Abbruch: Motorfehler\n");
            return -1;
        }
        usleep(MEAS_DELAY * 1000);
        set_frequency(mhz);
        current_swr = read_swr();
        if (current_swr < 0.0) {
            printf("DEBUG: Abbruch: Ungültiger SWR\n");
            return -1;
        }
        printf("DEBUG: SWR nach Schritt: %.2f (Best ever: %.2f)\n", current_swr, best_swr_ever);
        
        if (current_swr < best_swr_ever) {
            best_swr_ever = current_swr;
        }
        
        total_steps += step_size;

        // Vorsichtiger Modus nach gutem SWR
        if (best_swr_ever < SWR_GOOD_THRESHOLD && current_swr > best_swr_ever + 0.5) {
            printf("DEBUG: SWR steigt nach gutem Wert (%.2f) – aktiviere vorsichtigen Modus!\n", best_swr_ever);
            strcpy(direction, "tief");
            cautious_mode = 1;
        }

        if (current_swr < SWR_TARGET && current_swr > 0.0) {
            float best_swr = current_swr;
            long original_position = motor_position;
            float swr_hoch, swr_tief;
            printf("DEBUG: Feintuning SWR %.2f\n", best_swr);
            for (int i = 0; i < 15; i++) {
                if (send_motor_cmd("hoch", 1, 1) >= 0) {
                    swr_hoch = read_swr();
                    printf("DEBUG: SWR hoch (Versuch %d): %.2f\n", i + 1, swr_hoch);
                    if (swr_hoch < best_swr && swr_hoch > 0.0) {
                        best_swr = swr_hoch;
                        motor_position = original_position + i + 1;
                        continue;
                    }
                }
                if (send_motor_cmd("tief", 2, 1) >= 0) {
                    swr_tief = read_swr();
                    printf("DEBUG: SWR tief (Versuch %d): %.2f\n", i + 1, swr_tief);
                    if (swr_tief < best_swr && swr_tief > 0.0) {
                        best_swr = swr_tief;
                        motor_position = original_position - i - 1;
                        continue;
                    } else {
                        motor_position = original_position;
                        if (motor_position != original_position) send_motor_cmd("hoch", abs(motor_position - original_position), 1);
                    }
                }
                if (best_swr <= 1.5 || (swr_hoch >= best_swr && swr_tief >= best_swr)) break;
            }
            int steps = 0, rounds = 0;
            load_memory(mhz, &steps, &rounds);
            save_memory(mhz, steps, rounds);
            previous_mhz = mhz;
            strcpy(previous_band, get_band(mhz));
            printf("DEBUG: Tuning abgeschlossen, SWR: %.2f\n", best_swr);
            return 0;
        }
    }
    // Speichere auch bei Abbruch, wenn SWR gut war
    if (current_swr < SWR_TARGET + 1.0) {
        int steps = 0, rounds = 0;
        load_memory(mhz, &steps, &rounds);
        save_memory(mhz, total_steps, 0);
    }
    printf("DEBUG: Abbruch: Max. Schritte erreicht, SWR: %.2f\n", current_swr);
    previous_mhz = mhz;
    strcpy(previous_band, get_band(mhz));
    return -1;
}

enum MHD_Result http_handler(void* cls, struct MHD_Connection* connection, const char* url, const char* method, const char* version, const char* upload_data, size_t* upload_data_size, void** ptr) {
    static int dummy;
    if (*ptr != &dummy) {
        *ptr = &dummy;
        return MHD_YES;
    }
    if (*upload_data_size != 0) {
        *upload_data_size = 0;
        return MHD_YES;
    }
    if (strcmp(method, "GET") != 0) {
        const char* resp = "{\"error\": \"Nur GET-Methode erlaubt\"}";
        struct MHD_Response* response = MHD_create_response_from_buffer(strlen(resp), (void*)resp, MHD_RESPMEM_PERSISTENT);
        MHD_add_response_header(response, "Content-Type", "application/json");
        enum MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_METHOD_NOT_ALLOWED, response);
        MHD_destroy_response(response);
        return ret;
    }
    const char* freq_str = MHD_lookup_connection_value(connection, MHD_GET_ARGUMENT_KIND, "freq");
    if (!freq_str) {
        const char* resp = "{\"error\": \"Freq-Parameter fehlt\"}";
        struct MHD_Response* response = MHD_create_response_from_buffer(strlen(resp), (void*)resp, MHD_RESPMEM_PERSISTENT);
        MHD_add_response_header(response, "Content-Type", "application/json");
        enum MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_BAD_REQUEST, response);
        MHD_destroy_response(response);
        return ret;
    }
    double mhz = atof(freq_str);
    if (mhz < MIN_FREQ || mhz > MAX_FREQ) {
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"error\": \"Freq außerhalb %.1f-%.1f MHz\"}", MIN_FREQ, MAX_FREQ);
        struct MHD_Response* response = MHD_create_response_from_buffer(strlen(resp), (void*)resp, MHD_RESPMEM_PERSISTENT);
        MHD_add_response_header(response, "Content-Type", "application/json");
        enum MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_BAD_REQUEST, response);
        MHD_destroy_response(response);
        return ret;
    }
    if (check_nanovna() < 0) {
        const char* resp = "{\"error\": \"NanoVNA nicht verfügbar\"}";
        struct MHD_Response* response = MHD_create_response_from_buffer(strlen(resp), (void*)resp, MHD_RESPMEM_PERSISTENT);
        MHD_add_response_header(response, "Content-Type", "application/json");
        enum MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_INTERNAL_SERVER_ERROR, response);
        MHD_destroy_response(response);
        return ret;
    }
    char json[512];
    int status = auto_tune(mhz);
    float final_swr = read_swr();
    if (final_swr < 0.0) {
        snprintf(json, sizeof(json), "{\"status\": \"failed\", \"freq\": %.4f, \"swr\": null, \"band\": \"%s\", \"motor_position\": %ld, \"error\": \"SWR-Messung fehlgeschlagen\"}",
                 mhz, get_band(mhz), motor_position);
    } else {
        snprintf(json, sizeof(json), "{\"status\": \"%s\", \"freq\": %.4f, \"swr\": %.2f, \"band\": \"%s\", \"motor_position\": %ld}",
                 (status == 0) ? "success" : "failed", mhz, final_swr, get_band(mhz), motor_position);
    }
    struct MHD_Response* response = MHD_create_response_from_buffer(strlen(json), (void*)json, MHD_RESPMEM_PERSISTENT);
    MHD_add_response_header(response, "Content-Type", "application/json");
    enum MHD_Result ret = MHD_queue_response(connection, (status == 0 && final_swr >= 0.0) ? MHD_HTTP_OK : MHD_HTTP_INTERNAL_SERVER_ERROR, response);
    MHD_destroy_response(response);
    printf("DEBUG: HTTP-Antwort: %s\n", json);

    // Speichere bei Erfolg und gutem SWR
    if (status == 0 && final_swr > 0.0 && final_swr < SWR_TARGET + 1.0) {
        int steps = 0, rounds = 0;
        load_memory(mhz, &steps, &rounds);
        save_memory(mhz, steps, rounds);
    }
    return ret;
}

int main() {
    if (open_serial() < 0) {
        fprintf(stderr, "Serial-Init fehlgeschlagen\n");
        return 1;
    }
    if (check_nanovna() < 0) {
        fprintf(stderr, "NanoVNA nicht verfügbar\n");
        close(serial_fd);
        return 1;
    }
    send_serial_cmd("help");
    struct MHD_Daemon* daemon = MHD_start_daemon(MHD_USE_THREAD_PER_CONNECTION, HTTP_PORT, NULL, NULL, &http_handler, NULL, MHD_OPTION_END);
    if (!daemon) {
        fprintf(stderr, "HTTP-Server-Start fehlgeschlagen\n");
        close(serial_fd);
        return 1;
    }
    printf("Tuner-Server läuft auf http://0.0.0.0:%d\n", HTTP_PORT);
    getchar();
    MHD_stop_daemon(daemon);
    close(serial_fd);
    return 0;
}
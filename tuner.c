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

#define SERIAL_PORT "/dev/ttyACM0"
#define BAUDRATE B9600
#define ESP32_IP "192.168.1.113"
#define ESP32_PORT 75
#define HTTP_PORT 8080
#define MAX_STEPS 1000
#define STEP_SIZE 10
#define MEAS_DELAY 1000

int serial_fd = -1;
char esp32_ip[16] = ESP32_IP;

// Funktionsdeklarationen
int open_serial(void);
void send_serial_cmd(const char* cmd);
float read_swr(void);
void set_frequency(double mhz);
int send_motor_cmd(const char* direction, int steps);
int auto_tune(double mhz);
enum MHD_Result http_handler(void* cls, struct MHD_Connection* connection, const char* url, 
                            const char* method, const char* version, const char* upload_data, 
                            size_t* upload_data_size, void** ptr);

int open_serial() {
    printf("Öffne Serial-Port %s...\n", SERIAL_PORT);
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("Fehler beim Öffnen des Serial-Ports");
        return -1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Fehler bei tcgetattr");
        return -1;
    }
    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);
    cfmakeraw(&tty);
    tty.c_cflag = (CLOCAL | CREAD | CS8);
    tty.c_iflag &= ~ICRNL;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 10; // 1 Sekunde Timeout
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Fehler bei tcsetattr");
        return -1;
    }
    printf("Serial-Port erfolgreich geöffnet\n");
    tcflush(serial_fd, TCIOFLUSH); // Puffer leeren
    return 0;
}

void send_serial_cmd(const char* cmd) {
    if (serial_fd < 0) {
        printf("Serial-Port nicht geöffnet für Befehl: %s\n", cmd);
        return;
    }
    printf("Sende Serial-Befehl: %s\n", cmd);
    tcflush(serial_fd, TCOFLUSH); // Ausgangspuffer leeren
    write(serial_fd, cmd, strlen(cmd));
    write(serial_fd, "\r\n", 2);
    usleep(300000); // Erhöhte Wartezeit
}

float read_swr() {
    if (serial_fd < 0) {
        printf("Serial-Port nicht geöffnet für read_swr\n");
        return -1.0;
    }
    tcflush(serial_fd, TCIOFLUSH); // Beide Puffer leeren
    send_serial_cmd("pause");
    send_serial_cmd("data 0");
    char buf[256] = {0};
    float swr = -1.0;
    int max_attempts = 10; // Mehr Versuche
    for (int i = 0; i < max_attempts; i++) {
        int n = read(serial_fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            printf("Empfangene Daten von NanoVNA: %s\n", buf);
            float real, imag;
            char* line = strtok(buf, "\n");
            while (line) {
                if (sscanf(line, "%f %f", &real, &imag) == 2) {
                    float mag = sqrt(real * real + imag * imag);
                    if (mag < 1.0 && mag > 0.0) {
                        swr = (1.0 + mag) / (1.0 - mag + 1e-6);
                        printf("Berechnetes SWR: %.2f (Real: %f, Imag: %f, |S11|: %f)\n", swr, real, imag, mag);
                        send_serial_cmd("resume");
                        return swr;
                    } else {
                        printf("Ungültiger S11-Betrag: %f (Real: %f, Imag: %f)\n", mag, real, imag);
                    }
                } else {
                    printf("Fehler beim Parsen von S11-Daten: %s\n", line);
                }
                line = strtok(NULL, "\n");
            }
        } else {
            printf("Keine Daten von NanoVNA empfangen (Versuch %d/%d)\n", i + 1, max_attempts);
        }
        usleep(500000); // Erhöhte Wartezeit
    }
    send_serial_cmd("resume");
    return swr;
}

void set_frequency(double mhz) {
    long long hz = (long long)(mhz * 1000000.0);
    char cmd[64];
    send_serial_cmd("pause");
    tcflush(serial_fd, TCIOFLUSH); // Beide Puffer leeren
    snprintf(cmd, sizeof(cmd), "freq %lld", hz);
    send_serial_cmd(cmd);
    send_serial_cmd("trace 0 SWR CH0");
    snprintf(cmd, sizeof(cmd), "sweep %lld %lld 1", hz, hz);
    send_serial_cmd(cmd);
    usleep(MEAS_DELAY * 1000);
    send_serial_cmd("resume");
    printf("Frequenz auf %.4f MHz gesetzt\n", mhz);
}
int send_motor_cmd(const char* direction, int steps) {
    printf("Sende Motor-Befehl: %s,%d\n", direction, steps);
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Fehler beim Erstellen des Sockets");
        return -1;
    }
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(ESP32_PORT);
    inet_pton(AF_INET, esp32_ip, &addr.sin_addr);
    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Fehler bei der Verbindung zum ESP32");
        close(sock);
        return -1;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "%s,%d\n", direction, steps);
    write(sock, buf, strlen(buf));
    usleep(100000);
    close(sock);
    printf("Motor-Befehl gesendet\n");
    return 0;
}

int auto_tune(double mhz) {
    printf("Starte auto_tune für %.4f MHz\n", mhz);
    set_frequency(mhz);
    float current_swr = read_swr();
    if (current_swr < 2.0 && current_swr > 0.0) {
        printf("Bereits getunt: SWR = %.2f\n", current_swr);
        return 0;
    }
    printf("Start Tuning für %.4f MHz. Initial SWR: %.2f\n", mhz, current_swr);

    int direction = 1;
    int total_steps = 0;
    float prev_swr = current_swr;
    char dir_str[8];

    while (total_steps < MAX_STEPS) {
        strcpy(dir_str, (direction > 0) ? "hoch" : "tief");
        if (send_motor_cmd(dir_str, STEP_SIZE) < 0) {
            printf("ESP32-Fehler!\n");
            return -1;
        }
        usleep(MEAS_DELAY * 1000);
        set_frequency(mhz);
        current_swr = read_swr();
        total_steps += STEP_SIZE;

        printf("Schritt %d (%s): SWR = %.2f\n", total_steps, dir_str, current_swr);

        if (current_swr < 2.0 && current_swr > 0.0) {
            printf("Getunt! Finale SWR: %.2f\n", current_swr);
            return 0;
        }

        if (current_swr > prev_swr || current_swr < 0.0) {
            direction = -direction;
            prev_swr = current_swr < 0.0 ? 999.0 : current_swr;
        } else {
            prev_swr = current_swr;
        }
    }
    printf("Tuning fehlgeschlagen: Max. Schritte erreicht. SWR: %.2f\n", current_swr);
    return -1;
}

enum MHD_Result http_handler(void* cls, struct MHD_Connection* connection, const char* url, 
                            const char* method, const char* version, const char* upload_data, 
                            size_t* upload_data_size, void** ptr) {
    static int dummy;
    if (*ptr != &dummy) {
        *ptr = &dummy;
        return MHD_YES;
    }
    if (*upload_data_size != 0) {
        *upload_data_size = 0;
        return MHD_YES;
    }
    printf("HTTP-Anfrage: URL=%s, Method=%s\n", url, method);
    if (strcmp(method, "GET") != 0) {
        const char* resp = "{\"error\": \"Nur GET-Methode erlaubt\"}";
        struct MHD_Response* response = MHD_create_response_from_buffer(strlen(resp), (void*)resp, MHD_RESPMEM_PERSISTENT);
        MHD_add_response_header(response, "Content-Type", "application/json");
        enum MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_METHOD_NOT_ALLOWED, response);
        MHD_destroy_response(response);
        printf("Antwort: %s\n", resp);
        return ret;
    }

    const char* freq_str = MHD_lookup_connection_value(connection, MHD_GET_ARGUMENT_KIND, "freq");
    if (!freq_str) {
        const char* resp = "{\"error\": \"Freq-Parameter fehlt (z.B. ?freq=3.6855)\"}";
        struct MHD_Response* response = MHD_create_response_from_buffer(strlen(resp), (void*)resp, MHD_RESPMEM_PERSISTENT);
        MHD_add_response_header(response, "Content-Type", "application/json");
        enum MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_BAD_REQUEST, response);
        MHD_destroy_response(response);
        printf("Antwort: %s\n", resp);
        return ret;
    }

    double mhz = atof(freq_str);
    printf("Gelesene Frequenz: %.4f MHz\n", mhz);
    if (mhz < 1.5 || mhz > 10.2) {
        const char* resp = "{\"error\": \"Freq außerhalb 1.5-10.2 MHz\"}";
        struct MHD_Response* response = MHD_create_response_from_buffer(strlen(resp), (void*)resp, MHD_RESPMEM_PERSISTENT);
        MHD_add_response_header(response, "Content-Type", "application/json");
        enum MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_BAD_REQUEST, response);
        MHD_destroy_response(response);
        printf("Antwort: %s\n", resp);
        return ret;
    }

    char json[256];
    int status = auto_tune(mhz);
    float final_swr = read_swr();
    snprintf(json, sizeof(json), "{\"status\": \"%s\", \"freq\": %.4f, \"swr\": %.2f}", 
             (status == 0) ? "success" : "failed", mhz, final_swr);
    printf("Antwort JSON: %s\n", json);

    struct MHD_Response* response = MHD_create_response_from_buffer(strlen(json), (void*)json, MHD_RESPMEM_PERSISTENT);
    MHD_add_response_header(response, "Content-Type", "application/json");
    enum MHD_Result ret = MHD_queue_response(connection, (status == 0) ? MHD_HTTP_OK : MHD_HTTP_INTERNAL_SERVER_ERROR, response);
    MHD_destroy_response(response);
    return ret;
}

int main() {
    if (open_serial() < 0) {
        fprintf(stderr, "Serial-Init fehlgeschlagen. Überprüfe Port: %s\n", SERIAL_PORT);
        return 1;
    }
    send_serial_cmd("help");

    struct MHD_Daemon* daemon = MHD_start_daemon(MHD_USE_THREAD_PER_CONNECTION, HTTP_PORT, NULL, NULL,
                                                 &http_handler, NULL, MHD_OPTION_END);
    if (!daemon) {
        fprintf(stderr, "HTTP-Server-Start fehlgeschlagen\n");
        close(serial_fd);
        return 1;
    }

    printf("Tuner-Server läuft auf http://0.0.0.0:%d\n", HTTP_PORT);
    printf("ESP32-IP: %s:%d\n", esp32_ip, ESP32_PORT);
    printf("Drücke Enter zum Beenden...\n");
    getchar();

    MHD_stop_daemon(daemon);
    close(serial_fd);
    return 0;
}
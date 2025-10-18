**Antenna Tuner Controller**

**Einleitung**

Dieses Programm ist ein intelligenter Antennentuner-Controller für Amateurfunk-Anwendungen. Es steuert einen Schrittmotor über einen ESP32 (via TCP) und misst das SWR (Stehwellenverhältnis) mit einem NanoVNA. Die Tuner-Position wird automatisch optimiert basierend auf der Frequenz, Interpolation und Echtzeit-SWR-Messungen. Es ist speziell für Bänder wie 160m, 60m und 40m angepasst, mit band-spezifischen Parametern für grobe und feine Bewegungen.

**Features**

* **Automatisches Tuning**: Berechnet Zielposition, fährt grob hin und scannt fein bis zum besten SWR.
* **Band-spezifische Optimierung**:
  + 160m (1.8–2.0 MHz): 90% grobe Bewegung, feiner Scan (2 Schritte), Ziel-SWR 3.0.
  + 60m (5.0–5.5 MHz): 15% grobe Bewegung.
  + 40m (7.0–7.3 MHz): 50% grobe Bewegung.
  + Andere Bänder: Default 75%.
* **Persistente TCP-Verbindung** zum ESP32: Öffnet einmal und hält sie während des Tunings.
* **CSV-Logging**: Speichert beste Positionen in tuner\_positions.csv für schnelle Wiederverwendung.
* **Fehlerbehandlung**: Automatisches Reconnect zu NanoVNA und ESP32.
* **Konfigurierbar**: Alle Parameter über #define anpassbar (neu kompilieren nach Änderung).
* **Debug-Ausgaben**: Detaillierte Logs für Befehle und Entscheidungen.

**Hardware-Anforderungen**

* Host-System: Raspberry Pi oder Linux-PC mit serieller Schnittstelle (z. B. /dev/ttyACM0).
* NanoVNA: Via USB für SWR-Messungen.
* ESP32: Als TCP-Server (Port 75), steuert den Schrittmotor.
* Schrittmotor: Am Tuner montiert, kompatibel mit Befehlen wie "hoch", "tief".
* Netzwerk: Host und ESP32 im selben LAN.

**Installation**

1. Abhängigkeiten installieren (Debian/Raspberry Pi OS):

text

sudo apt update

sudo apt install gcc libpthread-stubs0-dev libm-dev

1. Code speichern: Kopiere tuner.c in ein Verzeichnis.
2. Kompilieren:

text

gcc -o tuner tuner.c -lpthread -lm

1. Ausführen (Beispiel: 1.885 MHz):

text

./tuner 1.885

**Konfiguration**

Ändere die #define-Direktiven in tuner.c und kompiliere neu.

**Wichtige Parameter**

* ESP32\_IP: IP des ESP32 (Standard: "192.168.1.29").
* SERIAL\_PORT: NanoVNA-Port (Standard: "/dev/ttyACM0").
* GROB\_PROZENT\_160M: Grober Anteil für 160m (0.90).
* GROB\_PROZENT\_60M: Für 60m (0.15).
* GROB\_PROZENT\_40M: Für 40m (0.50).
* SCAN\_STEP\_SIZE\_160M: Feine Schritte für 160m (2).
* NO\_IMPROVEMENT\_LIMIT\_160M: Stopp nach Schritten ohne Besserung (50).
* SWR\_TARGET\_160M: Ziel-SWR für 160m (3.0).
* CSV\_FILE: Speicherort der CSV (tuner\_positions.csv).

**Interpolationstabelle**

Bearbeite freq\_points[] für deine Kalibrierung:

text

{1.8, 0000}, {1.85, 920}, // etc.

**Workflow**

1. Lädt letzte Position aus CSV (bei gutem SWR).
2. Berechnet Ziel via Interpolation.
3. Grobe Bewegung (band-spezifisch, z. B. 90% für 160m).
4. Feintuning: Scannt, misst SWR, stoppt bei Optimum oder Stillstand, fährt zurück.
5. Speichert Ergebnis in CSV.

**Beispiel-Log (160m)**

text

INFO: Tuning für 1.8850 MHz

INFO: Grobe Bewegung: 638 Schritte hoch (90% von 709)

INFO: Starte Scan in Richtung hoch mit Schrittgröße 2

INFO: Scan Schritt 98: SWR 1.86 (Pos 709)

INFO: Besser! Neues bestes SWR 1.86 bei Pos 709

INFO: Ziel-SWR erreicht – stoppe

INFO: Position gespeichert in CSV: 1.8850 MHz bei Pos 709 mit SWR 1.86

Tuning erfolgreich

**Fehlerbehebung**

* NanoVNA nicht erreichbar: USB prüfen, Rechte setzen (sudo chmod 666 /dev/ttyACM0).
* ESP32 antwortet nicht: IP pingen, Firmware prüfen.
* SWR hoch: Antenne/Kabel testen.
* Motor still: Logs prüfen, Geschwindigkeit anpassen.
* CSV-Probleme: Datei löschen – wird neu erstellt.

**Lizenz**

MIT License – Freie Nutzung erlaubt.

**ESP**

* Arduino Software im esp direktory.

**Hardware**

Einzelheiten mailto:friedrich(at)riedhammer.net

73 – Viel Erfolg beim Tunen!
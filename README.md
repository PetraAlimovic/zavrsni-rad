
# astro_ws

ROS 2 workspace s paketima razvijenima u sklopu završnog rada za upravljanje mobilnim robotom pomoću gesta ruke i izbjegavanje prepreka.

Workspace sadrži sljedeće pakete:

- `hand_teleop`
- `hand_teleop_control`
- `hand_teleop_msgs`

Uz ovaj workspace potrebno je klonirati i repozitorij ASTRO robota s CRTA laboratorijskog GitHub repozitorija jer se u njemu nalazi paket `twist_mux` i ostale konfiguracije potrebne za pokretanje sustava.

---

## Pregled

Ovaj workspace sadrži implementaciju sustava za:

- prepoznavanje geste ruke pomoću MediaPipe Hands biblioteke
- procjenu trodimenzionalnog položaja šake pomoću Intel RealSense D435 kamere
- generiranje upravljačkih naredbi brzine za mobilnog robota
- automatsko izbjegavanje prepreka pomoću LiDAR senzora
- prosljeđivanje naredbi pomoću paketa `twist_mux`

Glavna launch datoteka je:

- `hand_teleop/launch/hand_teleop.launch.py`

Ona pokreće:

- `hand_capture_node`
- `hand_teleop_node`
- `obstacle_avoidance_node`
- `twist_mux` iz ASTRO repozitorija

---

## Preduvjeti

Prije pokretanja potrebno je imati instalirano sljedeće:

### Operacijski sustav i ROS 2
- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- `colcon`
- `rosdep`
- Python 3

### Python paketi
- `numpy`
- `opencv-python`
- `mediapipe`
- `pyrealsense2`
- ostale Python ovisnosti potrebne za ROS 2 pakete

### Intel RealSense kamera
Za rad s Intel RealSense D435 kamerom potrebno je imati instaliran:

- `pyrealsense2`

Upute za instalaciju i preuzimanje dostupne su na službenom Intel RealSense GitHub repozitoriju:

- https://github.com/IntelRealSense/librealsense

Tamo su navedeni svi koraci za instalaciju SDK-a, Python bindinga i potrebnih udev pravila.

### Dodatno
- USB 3.0 priključak za kameru
- Intel RealSense D435 kamera
- LiDAR senzor na robotu ASTRO
- ASTRO repozitorij s CRTA laboratorijskog GitHuba
- ispravno konfiguriran `twist_mux`

---

## Instalacija

### 1. Napravite workspace

Ako workspace još ne postoji, napravite direktorij i `src` mapu:

```bash
mkdir -p ~/astro_ws/src
cd ~/astro_ws/src
```

### 2. Klonirajte repozitorije

U `src` mapu klonirajte:

- svoj repozitorij
- ASTRO repozitorij s CRTA laboratorija

Primjer:

```bash
git clone https://github.com/PetraAlimovic/zavrsni-rad.git
git clone <URL_CRTA_ASTRO_REPOZITORIJA>
```

> Napomena: URL ASTRO repozitorija zamijenite stvarnim URL-om CRTA laboratorija.

Nakon kloniranja u `src` direktoriju trebali biste imati potrebne pakete za build.

---

## Kompajliranje workspacea

Nakon kloniranja repozitorija, vratite se u root workspacea i pokrenite build:

```bash
cd ~/astro_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Nakon uspješnog builda potrebno je sourceati workspace:

```bash
source install/setup.bash
```

Za praktičnost možete dodati sljedeće u `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/astro_ws/install/setup.bash
```

---

## Pokretanje sustava

Glavna launch datoteka pokreće cijeli sustav:

- `hand_capture_node`
- `hand_teleop_node`
- `obstacle_avoidance_node`
- `twist_mux`

Pokretanje:

```bash
ros2 launch hand_teleop hand_teleop.launch.py
```

Ako koristiš vlastitu konfiguraciju ili drugi naziv paketa, prilagodi naredbu prema strukturi workspacea.

---

## Struktura paketa

### `hand_teleop`
Sadrži glavnu logiku za:
- detekciju i klasifikaciju geste ruke
- procjenu položaja šake
- generiranje upravljačkih naredbi brzine
- komunikaciju s kamerom i grafičkim prikazom

### `hand_teleop_control`
Sadrži upravljačku logiku vezanu uz:
- pretvorbu informacija o ruci u brzine robota
- obradu i filtriranje naredbi
- dodatnu logiku za upravljanje sustavom

### `hand_teleop_msgs`
Sadrži prilagođene ROS 2 poruke korištene u sustavu, primjerice poruku za položaj i gestu ruke.

---

## Napomene

- Prije pokretanja provjerite da je Intel RealSense kamera ispravno spojena putem USB 3.0 sučelja.
- Ako kamera nije prepoznata, provjerite jesu li instalirani `pyrealsense2` i odgovarajuća `udev` pravila.
- Ako `twist_mux` nije pronađen, provjerite je li ASTRO repozitorij pravilno kloniran i buildan u istom workspaceu.
- Ako se koriste vlastiti putovi ili nazivi paketa, potrebno ih je uskladiti u launch datoteci.
- Ako sustav ne radi kako treba, provjerite jesu li svi ROS 2 workspaceovi sourceani u trenutnoj terminal sesiji.

---

## Korisni linkovi

- Intel RealSense SDK / librealsense: https://github.com/IntelRealSense/librealsense
- ROS 2 Humble dokumentacija: https://docs.ros.org/en/humble/
- CRTA laboratorij ASTRO repozitorij: <URL_CRTA_ASTRO_REPOZITORIJA>

---

## Autorica

Petra Alimović

## Završni rad

Ovaj workspace izrađen je u sklopu završnog rada na Fakultetu strojarstva i brodogradnje, Sveučilište u Zagrebu.

# Raspberry Startup Orchestration

Ovaj direktorij sadrzi skripte za Raspberry Pi koji nakon boot-a:

1. uspostavi UDP komunikaciju s `power_manager` ESP-om  
2. pokrene `scripts/tomo_launch.sh`  
3. posalje `ROS_READY`  
4. pokrene `scripts/start_micro_ros_agent.sh`  
5. ceka da su aktivni `/tomo/esp1_alive` i `/tomo/esp2_alive` publisheri (oba micro-ROS ESP-a spojena)  
6. posalje `AGENT_READY`  
7. stalno salje watchdog (`HEARTBEAT` + JSON status)  
8. na `SHUTDOWN` od ESP-a gasi sve fazno:
   - pokrece `stop_micro_ros_agent.sh` i salje `AGENT_STOPPED` (LED2 OFF)
   - ubija `tomo_launch.sh` i salje `ROS_STOPPED` (LED1 blink)
   - salje `LAST_MESSAGE`
   - salje `SAFE_TO_POWER_OFF` (ESP moze ugasiti LED1 i power railove)
   - svaka faza ceka ACK s ESP-a prije prelaska dalje

## Datoteke

- `power_orchestrator.py` - glavna logika (UDP + process manager)
- `run_power_orchestrator.sh` - wrapper koji source-a ROS env i pokrece Python skriptu
- `tomo-power-orchestrator.service` - systemd servis za auto-start
- `install_service.sh` - helper za instalaciju/enable servisa

## Preduvjeti na Raspberry Pi

1. Repo mora biti na:
   - `/home/tomo/tractor_tomo`
2. ROS 2 Jazzy instaliran:
   - `/opt/ros/jazzy/setup.bash`
3. Workspace build-an:
   - `cd ~/tractor_tomo/ros2_ws && colcon build --symlink-install`
4. Docker instaliran (micro-ROS agent koristi Docker image).

Ako Docker nije instaliran, instaliraj ga:

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker tomo
newgrp docker
docker --version
docker run --rm hello-world
```

Ako `newgrp docker` ne osvjezi sesiju kako treba, odjavi se i ponovno prijavi na Raspberry.

## Konfiguracija

U service datoteci podesi:

- `User=tomo`
- `WorkingDirectory=/home/tomo/tractor_tomo`
- `Environment=POWER_MANAGER_HOST=<IP_power_manager_esp>`

Datoteka:
- `scripts/raspberry/tomo-power-orchestrator.service`

## Instalacija servisa

```bash
cd ~/tractor_tomo
chmod +x scripts/raspberry/install_service.sh
./scripts/raspberry/install_service.sh
```

Rucno (ako ne koristis helper):

```bash
sudo cp scripts/raspberry/tomo-power-orchestrator.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable tomo-power-orchestrator.service
sudo systemctl restart tomo-power-orchestrator.service
```

## Sudoers (za remote shutdown)

Ako zelis da `SHUTDOWN` poruka s ESP-a ugasi i Raspberry (`shutdown -h now`) bez password prompt-a:

```bash
sudo visudo -f /etc/sudoers.d/tomo-power
```

Dodaj:

```text
tomo ALL=(root) NOPASSWD: /sbin/shutdown
```

Ako to ne zelis, u service env postavi:

```text
Environment=ORCH_EXEC_SHUTDOWN=0
```

## Provjera

Logovi:

```bash
journalctl -u tomo-power-orchestrator.service -f
```

Status:

```bash
systemctl status tomo-power-orchestrator.service
```

## Ocekivano ponasanje LED/FSM

- nakon prvih heartbeat poruka s Raspberry -> LED1 blink (boot/wait)
- nakon `ROS_READY` -> LED1 stalno ON
- nakon starta agenta, dok nema micro-ROS ESP -> LED2 blink
- kad su `/tomo/esp1_alive` i `/tomo/esp2_alive` aktivni i poslan `AGENT_READY` -> LED2 stalno ON
- ako watchdog/agent padne -> povratak u `WAITING_AGENT_READY` (LED2 opet blink)
- pri shutdown handshaku:
  - nakon `AGENT_STOPPED` -> LED2 OFF
  - nakon `ROS_STOPPED` -> LED1 blink
  - nakon `LAST_MESSAGE` -> LED1 OFF
  - ACK poruke s ESP-a: `ACK_AGENT_STOPPED`, `ACK_ROS_STOPPED`, `ACK_LAST_MESSAGE`, `ACK_SAFE_TO_POWER_OFF`
- na E-stop -> `RELAY_CH4_POWER_OUTPUTS` se odmah gasi, power manager ESP ostaje aktivan.

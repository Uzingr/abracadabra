# abracadabra

### Installare ROS su Lattepanda
### 1.Configurare MAVROS su Lattepanda
```bash
sudo apt-get update
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
```
##### P.S. Cambiare noetic con Humble
Installiamo i file di configurazione e i plugin MAVLink
```bash
sudo apt-get install ros-noetic-mavros-msgs
rosrun mavros install_geographiclib_datasets.sh
```
#### 2. Configurare la comunicazione tra Pixhwak e Lattepanda con file di lancio mavros.launch
#### 3. Scrivere un nodo ROS per trasferire le coordinate -> nodo ros python
#### 4. Configurare PX$ per utilizzare i dati di posizione
assicurati che la configurazione del firmware PX4 sulla Pixhawk sia impostata per accettare i dati di posizione esterni. Potrebbe essere necessario configurare i parametri EKF (Extended Kalman Filter) per utilizzare una sorgente di posizione esterna.
#### 5. Eseguire tutto insieme
- Avvia il Vicon-bridge per pubblicare le coordinate.
- Avvia MAVROS sul Lattepanda.
- Esegui il nodo ROS che trasferisce le coordinate dal Vicon a MAVROS.

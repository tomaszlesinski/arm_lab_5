# ARM Lab 05 – "Robot poszukujący"


Celem ćwiczenia było zaimplementowanie i przetestowanie, autonomicznego systemu poszukiwania obiektów dla
robota mobilnego TurtleBot3 Waffle w symulacyjnym środowisku mieszkania (arm_house.world)

---

## Uruchomienie

### 1. Uruchomienie kontenera
```bash
./arm_05_build.sh
./arm_05_run_gpu_nvidia.sh
```

### 2. Kompilacja pakietów
```bash
cd /arm_ws
colcon build --packages-select search_interfaces search_robot
source install/setup.bash
```

### 3. Start symulacji (Terminal 1)
```bash
ros2 launch arm05_sim world.launch.py
```

### 4. Start serwera poszukiwania (Terminal 2)
```bash
docker exec -it ARM_05 bash
ros2 launch search_robot search.launch.py
```

### 5. Start klienta akcji (Terminal 3)
```bash
docker exec -it ARM_05 bash
ros2 run search_robot search_client
```

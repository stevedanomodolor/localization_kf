# Localization KF  

This repository contains an implementation of a Kalman Filter for localization.  

---

## How to Run  

### 1. Install Docker  
Follow the official installation guide:  
👉 [Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)  

### 2. Pull the Docker Image  
```bash
docker pull stevedan/localization-kf:latest
```  

### 3. Run the Container (Terminal 1)  
```bash
docker run -it --rm \
    --name localization-kf \
    -e QT_X11_NO_MITSHM=1 \
    -e DISPLAY=$DISPLAY \
    --gpus=all \
    --network=host \
    --privileged \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --volume=<path-to-localization-kf>/localization_kf/localization_kf:/opt/overlay_ws/src \
    stevedan/localization-kf:latest bash
```  

Inside the container, launch the PX4 simulation:  
```bash
source /entrypoint
ros2 launch px4_sim px4_sim.launch.py
```  

### 4. Launch Navigator + EKF Nodes (Terminal 2)  
In another terminal:  
```bash
docker exec -it localization-kf bash
ros2 launch localization_kf_bringup bringup.launch.py
```  

---

## How to Build the Docker Image  
If you need to rebuild the image from source:  
```bash
docker build -f Docker/Dockerfile -t stevedan/localization-kf:latest .
```  

---

## References  
For details on the mathematical relationship between Euler angles and body rates, see:  
📄 [Relationship Between Euler Angles and Body Rates (BYU)](https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub)  
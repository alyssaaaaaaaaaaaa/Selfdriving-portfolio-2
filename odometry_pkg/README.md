# Technisch Verslag – Portfolio 2  
## Self-Driving Vehicles – Odometry, Visual SLAM en Sensor Fusion

---

# 1. Inleiding

In deze opdracht is een ROS2-gebaseerd systeem ontwikkeld voor een autonome Duckiebot-simulatie.  
Het doel van de opdracht was om drie onderdelen van autonome navigatie te implementeren:

1. Odometry op basis van encoderdata  
2. Vision-based monocular SLAM  
3. Sensor fusion van meerdere databronnen  

Het systeem is volledig ontwikkeld in Python met ROS2 nodes.  
De communicatie tussen de verschillende onderdelen verloopt via ROS2 topics.

Tijdens de ontwikkeling is gebruikgemaakt van:
- ROS2
- Python
- OpenCV
- cv_bridge
- NumPy
- Duckietown tooling

---

# 2. Systeemarchitectuur

Het systeem bestaat uit meerdere ROS2 nodes die onderling communiceren.

## Overzicht van de nodes

| Node | Functie |
|---|---|
| `encoder_sim_node` | Simuleert encoder ticks |
| `odometry_node` | Berekent positie uit encoderdata |
| `camera_sim_node` | Simuleert camerabeelden |
| `slam_node` | Detecteert en volgt features |
| `fusion_node` | Combineert odometry en visual motion |

---

## ROS2 Topics

| Topic | Beschrijving |
|---|---|
| `/wheel_encoders` | Encoderdata |
| `/odometry` | Positie uit encoders |
| `/camera/image_raw` | Camerabeelden |
| `/visual_motion` | Visuele bewegingsschatting |
| `/slam/feature_image` | Beeld met features |
| `/fused_pose` | Gefuseerde positie |

---

# 3. Taak 1 – Odometry

## Doel

Het doel van taak 1 was het berekenen van de positie van de robot op basis van wielencoderdata.

---

# Implementatie

Voor deze taak zijn twee nodes gemaakt:

- `encoder_sim_node`
- `odometry_node`

De encoder simulator genereert continu linker- en rechter wielticks.  
Deze waarden worden gepubliceerd naar het ROS2 topic:

```python
'/wheel_encoders'
```

De odometry node leest deze waarden uit en berekent:
- x positie
- y positie
- rotatiehoek (`theta`)

Hiervoor is differential drive kinematics gebruikt.

---

# Encoder Simulator

## Gebruikte code

```python
self.left_ticks_per_sec = 60.0
self.right_ticks_per_sec = 80.0
```

Hiermee worden verschillende snelheden voor de linker- en rechterwielen gesimuleerd.

---

## Publiceren van encoderdata

```python
msg.data = [self.left_ticks, self.right_ticks]
self.publisher_.publish(msg)
```

---

# Odometry Berekening

De odometry node berekent de positie op basis van de verandering in encoderwaarden.

## Gebruikte code

```python
delta_left_ticks = left_ticks - self.prev_left_ticks
delta_right_ticks = right_ticks - self.prev_right_ticks
```

Daarna wordt de afgelegde afstand berekend:

```python
left_distance = 2 * math.pi * self.wheel_radius * (delta_left_ticks / self.ticks_per_revolution)
right_distance = 2 * math.pi * self.wheel_radius * (delta_right_ticks / self.ticks_per_revolution)
```

Vervolgens wordt de nieuwe positie berekend:

```python
self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
self.theta += delta_theta
```

---

# Terminalstructuur tijdens demonstratie

## Terminal 1

Encoder simulator starten:

```bash
ros2 run odometry_pkg encoder_sim_node
```

Deze terminal liet realtime encoder ticks zien.

---

## Terminal 2

Odometry node starten:

```bash
ros2 run odometry_pkg odometry_node
```

Deze terminal liet de berekende positie zien:

```python
Pose -> x: ..., y: ..., theta: ...
```

---

# Video taak 1

In de video van taak 1 is zichtbaar dat:
- encoderwaarden continu veranderen
- de odometry node nieuwe posities berekent
- de robotpositie realtime wordt geüpdatet

---

# Ontwerpkeuzes

Er is gekozen voor een eenvoudige encoder simulator in plaats van een volledige fysieke robot.

Voordelen hiervan:
- eenvoudiger debuggen
- stabielere ROS2 communicatie
- focus op de algoritmes

---

# Beperkingen

Odometry op basis van encoders heeft drift.

Kleine fouten stapelen zich op waardoor:
- positie onnauwkeurig wordt
- rotatiefouten ontstaan

---

# 4. Taak 2 – Vision-Based Monocular SLAM

## Doel

Het doel van taak 2 was:
- features detecteren
- features volgen
- visuele beweging berekenen

op basis van camerabeelden.

---

# Camera Simulator

Omdat de volledige Duckietown Matrix simulatie niet stabiel werkte op het systeem, is gekozen voor een eigen camerasimulator.

Deze node genereert eenvoudige vormen:
- vierkant
- cirkel
- lijn

---

# Gebruikte code

```python
img = np.ones((480, 640, 3), dtype=np.uint8) * 255
```

Hiermee wordt een witte achtergrond gemaakt.

Daarna worden vormen toegevoegd:

```python
cv2.rectangle(img, ...)
cv2.circle(img, ...)
cv2.line(img, ...)
```

Deze vormen zorgen voor duidelijke hoeken en features.

---

# Waarom deze aanpak?

Aanvankelijk is geprobeerd om de Duckietown Matrix simulatie te gebruiken.  
Hierbij ontstonden problemen met:

- OpenGL rendering
- ROS GUI forwarding
- rqt_image_view
- Docker GUI ondersteuning

Daarom is gekozen voor een stabielere camerasimulator.

Voordelen:
- stabielere featuredetectie
- eenvoudiger debuggen
- minder afhankelijk van GPU rendering

---

# Feature Detectie

De SLAM node gebruikt OpenCV voor featuredetectie.

## Gebruikte code

```python
self.prev_points = cv2.goodFeaturesToTrack(
    gray,
    maxCorners=100,
    qualityLevel=0.3,
    minDistance=7,
    blockSize=7
)
```

Hiermee worden sterke hoeken gezocht.

---

# Optical Flow

Features worden gevolgd tussen frames met Lucas-Kanade optical flow.

## Gebruikte code

```python
next_points, status, error = cv2.calcOpticalFlowPyrLK(
    self.prev_gray,
    gray,
    self.prev_points,
    None
)
```

---

# Visual Motion

De gemiddelde featurebeweging wordt gebruikt om een bewegingsschatting te maken.

## Gebruikte code

```python
dx = np.mean(good_new[:,0] - good_old[:,0])
dy = np.mean(good_new[:,1] - good_old[:,1])
```

Deze waarden worden gepubliceerd naar:

```python
'/visual_motion'
```

---

# Feature Visualisatie

Features worden visueel weergegeven met groene punten.

## Gebruikte code

```python
cv2.circle(frame, (int(x_new), int(y_new)), 5, (0,255,0), -1)
```

Hierdoor zijn de gevolgde punten zichtbaar in de video.

---

# Feature afbeelding opslaan

De feature afbeelding wordt opgeslagen.

## Gebruikte code

```python
cv2.imwrite('/tmp/slam_feature_image.png', frame)
```

---

# Terminalstructuur tijdens demonstratie

## Terminal 1

Camera simulator:

```bash
ros2 run odometry_pkg camera_sim_node
```

---

## Terminal 2

SLAM node:

```bash
ros2 run odometry_pkg slam_node
```

---

# Video taak 2

## Video 1 – Featuredetectie

In de video is zichtbaar dat:
- groene punten op hoeken verschijnen
- features realtime worden gevolgd

---

## Video 2 – Feature afbeelding

Hierin is zichtbaar dat:
- de feature afbeelding wordt opgeslagen
- de featuredetectie correct werkt

---

# Ontwerpkeuzes

Er is bewust gekozen voor:
- eenvoudige OpenCV methoden
- monocular vision
- lichte simulatie

in plaats van deep learning.

Hierdoor bleef het systeem:
- sneller
- stabieler
- beter uitlegbaar

---

# Beperkingen

Vision-based SLAM heeft beperkingen:
- weinig features → tracking mislukt
- snelle beweging → optical flow fouten
- weinig contrast → slechte detectie

---

# 5. Taak 3 – Sensor Fusion

## Doel

Het doel van taak 3 was het combineren van:
- odometry data
- visual SLAM data

tot één gecombineerde positie.

---

# Fusion Node

De fusion node leest data uit:
- `/odometry`
- `/visual_motion`

en combineert deze.

---

# Gebruikte code

```python
fused_msg.x = 0.7 * self.odom_x + 0.3 * self.visual_x
fused_msg.y = 0.7 * self.odom_y + 0.3 * self.visual_y
fused_msg.theta = 0.7 * self.odom_theta + 0.3 * self.visual_theta
```

Hierbij krijgt odometry meer gewicht omdat deze stabieler is.

---

# Publiceren van fused pose

```python
self.fused_pub.publish(fused_msg)
```

Publicatie gebeurt op:

```python
'/fused_pose'
```

---

# Terminalstructuur tijdens demonstratie

## Terminal 1

Encoder simulator:

```bash
ros2 run odometry_pkg encoder_sim_node
```

---

## Terminal 2

Odometry node:

```bash
ros2 run odometry_pkg odometry_node
```

---

## Terminal 3

Camera simulator:

```bash
ros2 run odometry_pkg camera_sim_node
```

---

## Terminal 4

SLAM node:

```bash
ros2 run odometry_pkg slam_node
```

---

## Terminal 5

Fusion node:

```bash
ros2 run odometry_pkg fusion_node
```

---

# Video taak 3

In de video is zichtbaar dat:
- alle nodes tegelijk draaien
- ROS2 topics actief communiceren
- fused pose continu verandert

De terminaloutput toont:
- fused x
- fused y
- fused theta

---

# Ontwerpkeuzes

Er is gekozen voor een eenvoudige gewogen gemiddelde sensor fusion.

Voordelen:
- eenvoudig uitlegbaar
- stabiel
- goed passend binnen de opdracht

Een complexere aanpak zoals een Extended Kalman Filter zou nauwkeuriger zijn, maar ook veel ingewikkelder.

---

# Beperkingen

De huidige sensor fusion:
- filtert ruis beperkt
- corrigeert drift niet volledig
- gebruikt geen probabilistische modellen

---

# 6. Conclusie

In deze opdracht is succesvol een volledige ROS2 pipeline ontwikkeld voor:
- odometry
- vision-based SLAM
- sensor fusion

Ondanks problemen met de Duckietown Matrix simulator is een stabiele oplossing gerealiseerd met:
- realtime communicatie
- feature tracking
- motion estimation
- sensor fusion

De demonstratievideo’s tonen aan dat alle onderdelen correct functioneren en succesvol samenwerken binnen ROS2.
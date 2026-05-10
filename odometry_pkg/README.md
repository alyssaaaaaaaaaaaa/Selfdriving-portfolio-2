# Taak 1 – Odometry

## Inleiding

Voor taak 1 is een ROS2-node ontwikkeld die de positie van een Duckiebot kan schatten op basis van wielencoderdata. Dit wordt odometry genoemd. Odometry betekent dat de robot zijn eigen beweging inschat door te kijken hoeveel de wielen hebben gedraaid.

De robotpose bestaat uit drie onderdelen:

- `x`: positie op de x-as
- `y`: positie op de y-as
- `theta`: oriëntatie/draaihoek van de robot

Het doel van deze taak was om de robotpositie continu te berekenen en te publiceren op een ROS2-topic.

---

# Doel van de opdracht

Het doel van taak 1 is:

- een ROS2-node maken voor odometry;
- wheel encoder data gebruiken als input;
- een differential drive model implementeren;
- de pose van de robot schatten als `(x, y, theta)`;
- de pose publiceren op een ROS2-topic;
- de pose continu updaten tijdens beweging.

---

# ROS2-package

Voor deze taak is een ROS2-package gemaakt met de naam:

```text
odometry_pkg
```

De belangrijkste bestanden zijn:

```text
Selfdriving-portfolio-2
├── odometry_pkg
│   ├── __init__.py
│   ├── encoder_sim_node.py
│   └── odometry_node.py
│
├── resource
│   └── odometry_pkg
│
├── package.xml
├── setup.cfg
└── setup.py
```

---

# Nodes

Het systeem bestaat uit twee ROS2-nodes.

## 1. encoder_sim_node.py

Deze node simuleert de wielencoderdata van de Duckiebot. Omdat er tijdens het ontwikkelen niet direct met een echte Duckiebot is gewerkt, wordt encoderdata nagebootst.

De node publiceert de encoderwaarden van het linker- en rechterwiel op het topic:

```text
/wheel_encoders
```

De data wordt gepubliceerd als een `Float32MultiArray`, waarbij:

```text
data[0] = linker encoder ticks
data[1] = rechter encoder ticks
```

Voorbeeld:

```text
left: 3708.00, right: 4944.00
```

---

## 2. odometry_node.py

Deze node ontvangt de encoderdata via het topic:

```text
/wheel_encoders
```

Daarna berekent de node de robotpose:

```text
x
y
theta
```

De berekende pose wordt gepubliceerd op het topic:

```text
/odometry
```

De output is een `Pose2D` message.

---

# Publisher en subscriber

In ROS2 communiceren nodes via topics.

In deze opdracht is gebruikgemaakt van:

## Publisher

De `encoder_sim_node.py` publiceert encoderdata:

```text
/wheel_encoders
```

## Subscriber

De `odometry_node.py` luistert naar dit topic en ontvangt de encoderdata.

## Publisher odometry

De `odometry_node.py` publiceert daarna de berekende pose op:

```text
/odometry
```

De datastroom is dus:

```text
encoder_sim_node.py
        ↓
/wheel_encoders
        ↓
odometry_node.py
        ↓
/odometry
```

---

# Systeemarchitectuur

Het systeem bestaat uit twee ROS2-nodes die communiceren via topics.

De `encoder_sim_node.py` simuleert wielencoderdata van de Duckiebot en publiceert deze data op het topic `/wheel_encoders`.

De `odometry_node.py` ontvangt deze encoderdata, verwerkt de wielbewegingen en berekent hiermee de geschatte robotpose `(x, y, theta)`.

De berekende pose wordt vervolgens gepubliceerd op het topic `/odometry`.

De communicatie tussen de nodes verloopt volledig via het ROS2 publisher/subscriber model.

Datastroom:

```text
encoder_sim_node.py
        ↓
/wheel_encoders
        ↓
odometry_node.py
        ↓
/odometry
```

---

# Differential drive model

De Duckiebot gebruikt twee aangedreven wielen: een linkerwiel en een rechterwiel. Dit wordt een differential drive robot genoemd.

Als beide wielen even snel draaien, rijdt de robot recht vooruit.

Als het rechterwiel sneller draait dan het linkerwiel, draait de robot naar links.

Als het linkerwiel sneller draait dan het rechterwiel, draait de robot naar rechts.

Daarom kan de positie van de robot worden geschat door het verschil tussen de linker- en rechterwielbeweging te gebruiken.

---

# Gebruikte parameters

In de odometry-node zijn de volgende robotparameters gebruikt:

```python
self.wheel_radius = 0.0318
self.wheel_base = 0.10
self.ticks_per_revolution = 20.0
```

Betekenis:

| Parameter | Betekenis |
|---|---|
| `wheel_radius` | straal van het wiel in meters |
| `wheel_base` | afstand tussen linker- en rechterwiel |
| `ticks_per_revolution` | aantal encoder ticks per volledige wielomwenteling |

---

# Berekening van afstand per wiel

Eerst wordt bepaald hoeveel ticks het linker- en rechterwiel sinds de vorige meting zijn veranderd:

```python
delta_left_ticks = left_ticks - self.prev_left_ticks
delta_right_ticks = right_ticks - self.prev_right_ticks
```

Daarna worden deze ticks omgerekend naar afstand:

```python
left_distance = 2 * math.pi * self.wheel_radius * (delta_left_ticks / self.ticks_per_revolution)
right_distance = 2 * math.pi * self.wheel_radius * (delta_right_ticks / self.ticks_per_revolution)
```

Hiermee wordt berekend hoeveel meter elk wiel heeft afgelegd.

---

# Berekening van verplaatsing en draaiing

De gemiddelde verplaatsing van de robot wordt berekend met:

```python
delta_s = (left_distance + right_distance) / 2.0
```

De verandering in oriëntatie wordt berekend met:

```python
delta_theta = (right_distance - left_distance) / self.wheel_base
```

Als `right_distance` en `left_distance` gelijk zijn, dan is `delta_theta` nul en rijdt de robot rechtdoor.

Als er verschil is tussen beide afstanden, draait de robot.

---

# Update van x, y en theta

Daarna wordt de pose van de robot bijgewerkt:

```python
self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
self.theta += delta_theta
```

Hiermee wordt berekend waar de robot naartoe is verplaatst.

De robot houdt dus continu zijn geschatte positie bij.

---

# Publiceren van de pose

De berekende pose wordt opgeslagen in een `Pose2D` message:

```python
pose_msg = Pose2D()
pose_msg.x = self.x
pose_msg.y = self.y
pose_msg.theta = self.theta
```

Daarna wordt deze gepubliceerd:

```python
self.pose_publisher.publish(pose_msg)
```

Hierdoor is de pose beschikbaar op het ROS2-topic:

```text
/odometry
```

---

# Ontwerpkeuzes en afwegingen

Tijdens de implementatie is gekozen voor een eenvoudige en overzichtelijke architectuur met twee losse ROS2-nodes.

Er is gebruikgemaakt van een gesimuleerde encoder-node omdat er tijdens het ontwikkelen niet continu een fysieke Duckiebot beschikbaar was. Hierdoor kon de odometry-node zelfstandig getest worden.

Er is gekozen voor een differential drive model omdat Duckiebots gebruikmaken van twee onafhankelijk aangedreven wielen. Dit model is relatief eenvoudig te implementeren en geschikt voor basisodometry.

Voor communicatie tussen de nodes is gekozen voor ROS2-topics, omdat dit de standaardmanier is waarop ROS-systemen data uitwisselen.

Daarnaast is gekozen voor het `Pose2D` message type omdat voor deze taak alleen de positie in een 2D-vlak nodig was.

---

# Beperkingen en faalscenario’s

Odometry op basis van wielencoders heeft enkele beperkingen.

Een belangrijk probleem is dat kleine meetfouten zich opstapelen over tijd. Hierdoor kan de geschatte positie langzaam afwijken van de werkelijke positie van de robot. Dit wordt drift genoemd.

Daarnaast gaat het model ervan uit dat de wielen altijd perfecte grip hebben. In de praktijk kan slip optreden, waardoor de berekeningen minder nauwkeurig worden.

Ook wordt er geen gebruikgemaakt van externe sensoren zoals camera’s of LiDAR om de positie te corrigeren. Hierdoor blijft de nauwkeurigheid beperkt bij langere ritten.

Verder is in deze implementatie gebruikgemaakt van gesimuleerde encoderdata. Hierdoor zijn onregelmatigheden van echte hardware niet volledig meegenomen.

Mogelijke faalscenario’s zijn:

- foutieve encoderwaarden;
- wegvallende topiccommunicatie;
- verkeerde wielparameters;
- onnauwkeurige metingen door slip;
- afwijkingen door afrondingsfouten.

Wanneer één van deze situaties optreedt, kan de geschatte robotpositie incorrect worden.

---

# Mogelijke verbeteringen

De nauwkeurigheid van het systeem kan verbeterd worden door extra sensoren toe te voegen, zoals een camera of IMU.

Daarnaast kan SLAM worden toegepast zodat de robot niet alleen zijn positie schat, maar ook een kaart van de omgeving opbouwt.

Ook zou visualisatie in RViz toegevoegd kunnen worden om de robotpositie live weer te geven.

Verder kan gebruikgemaakt worden van echte encoderdata van een fysieke Duckiebot in plaats van simulatie.

---

# Gebruikte software en technieken

Voor deze opdracht is gebruikgemaakt van:

| Technologie | Toepassing |
|---|---|
| Ubuntu Linux | Ontwikkelomgeving |
| ROS2 | Robotcommunicatie en nodes |
| Python | Implementatie van de nodes |
| VS Code | Code-editor |
| GitHub | Versiebeheer |
| Publisher/subscriber model | Communicatie tussen nodes |

---

# Uitvoeren van de code

Eerst wordt de ROS2-workspace gebouwd:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Daarna wordt de encoder simulator gestart:

```bash
ros2 run odometry_pkg encoder_sim_node
```

In een tweede terminal wordt de odometry-node gestart:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run odometry_pkg odometry_node
```

Om te controleren of de odometry-data wordt gepubliceerd:

```bash
ros2 topic echo /odometry
```

---

# Testen van het systeem

Tijdens het testen zijn beide nodes afzonderlijk gestart in aparte terminals.

De encoder simulator publiceerde continu encoderwaarden op het topic `/wheel_encoders`.

De odometry-node ontving deze waarden en berekende live de robotpose.

Met het volgende commando werd gecontroleerd of de pose correct gepubliceerd werd:

```bash
ros2 topic echo /odometry
```

Tijdens het testen veranderden de waarden van `x`, `y` en `theta` continu, wat bevestigt dat de odometryberekeningen correct uitgevoerd werden.

Voorbeeld output:

```text
x: 0.033
y: 0.001
theta: 232.573
```

---

# Resultaat

De odometry-node voldoet aan de eisen van taak 1:

| Eis | Voldaan |
|---|---|
| ROS-node voor odometry | Ja |
| Wheel encoder data als input | Ja |
| Differential drive model | Ja |
| Pose schatten als x, y en theta | Ja |
| Publiceren op `/odometry` | Ja |
| Continu updaten tijdens beweging | Ja |

---

# Conclusie

In deze opdracht is succesvol een ROS2 odometry-systeem ontwikkeld voor een Duckiebot.

Met behulp van wielencoderdata en een differential drive model kan de robot zijn positie en oriëntatie schatten. De pose wordt live gepubliceerd op het topic `/odometry`.

De implementatie voldoet aan de eisen van taak 1 en vormt een basis voor verdere uitbreiding met SLAM en mapping.
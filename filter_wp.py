import json
import geopy.distance


CORRECTION_HEIGHT = 47.2
WP_PER_M = 1

standard_vel = 4.0
standard_exa = 1.0

gate_vel = 0.5
gate_exa = 0.2
gate_rad = 1.0
gate_dec_rad = 5.0

with open('/home/julien/RoboDroneRace/KnightsOfNyquistRDR/Path.json') as f:
    data = json.load(f)
with open('/home/julien/RoboDroneRace/KnightsOfNyquistRDR/emptyMission.json') as f:
    emptyMission = json.load(f)


def distanceBetween(p1, p2):
    lat1, lon1 = p1[0], p1[1]
    lat2, lon2 = p2[0], p2[1]
    return geopy.distance.vincenty((lat1, lon1), (lat2, lon2)).m

def velNexaAtDistance(dis):
    global standard_vel, standard_exa, gate_vel, gate_exa, gate_rad, gate_dec_rad

    if dis < gate_rad: return gate_vel, gate_exa
    if dis > gate_dec_rad: return standard_vel, standard_exa
    a = (standard_vel-gate_vel) / (gate_dec_rad-gate_rad)
    b = 0.5 * (standard_vel + gate_vel - a * (gate_dec_rad-gate_rad))
    vel = (a*dis + b)
    c = (standard_exa-gate_exa) / (gate_dec_rad-gate_rad)
    d = 0.5 * (standard_exa + gate_exa - c * (gate_dec_rad-gate_rad))
    exa = (c*dis + d)
    return vel, exa


lastWP = [0,0]
waypoints = []  # [latitude, longitude, altitude, velocity, exactness]
for el in data["Waypoints"]:
    wpNew = [el["lat"], el["lon"], el["alt"]-CORRECTION_HEIGHT, el["vel"] if "vel" in el else standard_vel, el["exa"] if "exa" in el else standard_exa]

    if distanceBetween(lastWP, wpNew) > 1.0/WP_PER_M:
        waypoints.append(wpNew)
        lastWP = wpNew

gates = []
for el in data["Gates"]:
    gates.append([el["lat"], el["lon"], el["alt"]])



# adjust speed depending on distance to gate
for gate in gates:
    for i in range(0, len(waypoints)):
        waypoint = waypoints[i]
        distance = distanceBetween(gate, waypoint)
        vel, exa = velNexaAtDistance(distance)

        waypoint[3] = vel if waypoint[3] > vel else waypoint[3]
        waypoint[4] = exa if waypoint [4] > exa else waypoint[4]
        waypoints[i] = waypoint

def createSimpleItem(lat, lon, alt, exa, cmd, i, a, b, frame):
    item = {}
    item["AMSLAltAboveTerrain"] = None
    item["Altitude"] = alt
    item["AltitudeMode"] = 1
    item["autoContinue"] = True
    item["command"] = cmd
    item["doJumpId"] = i
    item["frame"] = frame
    item["params"] = [a, exa, b, None, lat, lon, alt]
    item["type"] = "SimpleItem"
    return item


def setVel(vel, i):
    return createSimpleItem(0, 0, 0, vel, 178, i, 1, -1, 2)


def createWP(lat, lon, alt, exa, i):
    return createSimpleItem(lat, lon, alt, exa, 16, i, 0, 0, 0)


def createTakeoff(lat, lon, alt, i):
    return createSimpleItem(lat, lon, alt, 0, 22, i, 15, 0, 0)


i = 1

items = []
items.append(createTakeoff(waypoints[0][0], waypoints[0][1], waypoints[0][2], i))
i = i + 1

for el in waypoints:# [latitude, longitude, altitude, velocity, exactness]
    item1 = createWP(el[0], el[1], el[2], el[4], i)
    i = i + 1
    item2 = setVel(el[3], i)
    i = i + 1
    items.append(item1)
    items.append(item2)

emptyMission["mission"]["items"] = items
emptyMission["mission"]["plannedHomePosition"] = [waypoints[0][0], waypoints[0][1], waypoints[0][2]]

# Store json to file
text_file = open("/home/julien/RoboDroneRace/KnightsOfNyquistRDR/filteredWaypoints.plan", "w")
text_file.write(json.dumps(emptyMission, indent=4))
text_file.close()

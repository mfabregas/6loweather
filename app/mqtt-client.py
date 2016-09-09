### Taken from https://pypi.python.org/pypi/paho-mqtt
### Requires Paho-MQTT package, install by:
### pip install paho-mqtt

import paho.mqtt.client as mqtt

# relayr broker is mqtt.relayr.io, mosquitto broker is test.mosquitto.org

MQTT_URL         = "test.mosquitto.org"
MQTT_TOPIC_EVENT = "/v1/my_user/data"
MQTT_TOPIC_CMD   = "/v1/my_user/cmd"
MQTT_USER        = ""
MQTT_PASSWD      = ""

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(MQTT_TOPIC_EVENT)
    print("Subscribed to " + MQTT_TOPIC_EVENT)
    client.subscribe(MQTT_TOPIC_CMD)
    print("Subscribed to " + MQTT_TOPIC_CMD)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print("connecting to " + MQTT_URL)
if MQTT_USER:
  client.username_pw_set(MQTT_USER, MQTT_PASSWD)
  print("User: " + MQTT_USER + " password: " + MQTT_PASSWD)

client.connect(MQTT_URL, 1883, 60)
client.loop_forever()

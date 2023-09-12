
#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/ardupilotmega/mavlink_msg_obstacle_distance_3d.h"
#include "HLK_LD2450.h"


HLK_LD2450 ld2450(&Serial1);

unsigned long previousMillis = 0;
const long interval = 100;

int target = 0;
unsigned char data_buffer[4] = {0};
int adjusted = 0;
int distance = 0;
int range = 0;
unsigned char CS;
uint8_t Index;
byte received;
char serial_buffer[15];

int ledState = LOW;


void setup()
{
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.setRX(4);
  Serial1.setTX(5);
  
  Serial2.begin(1500000); // FC
  Serial.begin(500000);
  ld2450.begin();

}


void loop()
{
  command_heartbeat();
  ld2450.read();
  command_Target1();
  command_Target2();
  command_Target3();
  command_print() ;
 
}


void command_heartbeat() {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
int type = MAV_TYPE_GROUND_ROVER;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 30;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(1, 196, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}

void command_Target1() {

int sysid = 1;                     //< The component sending the message.
int compid = 196;
uint32_t time_boot_ms = 0;         //    ms Timestamp (time since system boot).
uint8_t sensor_type = 0;           //    MAV_DISTANCE_SENSOR Class id of the distance sensor type.
const uint16_t frame = MAV_FRAME_BODY_FRD ;//   MAV_FRAME Coordinate frame of reference.
uint16_t obstacle_id = 65535;       //    Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is unknown or cannot be determined.
float x = ld2450.getTargetX(0);                           //m   X position of the obstacle.
float y = ld2450.getTargetY(0);                           //m   Y position of the obstacle.
float z = 0;                           //m   Z position of the obstacle.
float min_distance = 0 ;         //m   Minimum distance the sensor can measure.
float max_distance = 5;         //m   Maximum distance the sensor can measure.


// Initialize the required buffers
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
int type = MAV_TYPE_GROUND_ROVER;

// Pack the message
mavlink_msg_obstacle_distance_3d_pack(sysid, compid, &msg, time_boot_ms, sensor_type, frame, obstacle_id, x, y, z, min_distance, max_distance);
uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
Serial1.write(buf, len);
}

void command_Target2() {

int sysid = 1;                     //< The component sending the message.
int compid = 196;
uint32_t time_boot_ms = 0;         //    ms Timestamp (time since system boot).
uint8_t sensor_type = 0;           //    MAV_DISTANCE_SENSOR Class id of the distance sensor type.
const uint16_t frame = MAV_FRAME_BODY_FRD ;//   MAV_FRAME Coordinate frame of reference.
uint16_t obstacle_id = 65535;       //    Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is unknown or cannot be determined.
float x = ld2450.getTargetX(1);                           //m   X position of the obstacle.
float y = ld2450.getTargetY(1);                           //m   Y position of the obstacle.
float z = 0;                           //m   Z position of the obstacle.
float min_distance = 0 ;         //m   Minimum distance the sensor can measure.
float max_distance = 5;         //m   Maximum distance the sensor can measure.

// Initialize the required buffers
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

// Pack the message
mavlink_msg_obstacle_distance_3d_pack(sysid, compid, &msg, time_boot_ms, sensor_type, frame, obstacle_id, x, y, z, min_distance, max_distance);
uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
Serial1.write(buf, len);
}

void command_Target3() {
  
int sysid = 1;                     //< The component sending the message.
int compid = 196;
uint32_t time_boot_ms = 0;         //    ms Timestamp (time since system boot).
uint8_t sensor_type = 0;           //    MAV_DISTANCE_SENSOR Class id of the distance sensor type.
const uint16_t frame = MAV_FRAME_BODY_FRD ;//   MAV_FRAME Coordinate frame of reference.
uint16_t obstacle_id = 65535;       //    Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is unknown or cannot be determined.
float x = ld2450.getTargetX(2);                           //m   X position of the obstacle.
float y = ld2450.getTargetY(2);                           //m   Y position of the obstacle.
float z = 0;                           //m   Z position of the obstacle.
float min_distance = 0 ;         //m   Minimum distance the sensor can measure.
float max_distance = 5;         //m   Maximum distance the sensor can measure.


// Initialize the required buffers
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

// Pack the message
mavlink_msg_obstacle_distance_3d_pack(sysid, compid, &msg, time_boot_ms, sensor_type, frame, obstacle_id, x, y, z, min_distance, max_distance);
uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
Serial1.write(buf, len);
}



void command_print() {
 Serial.println("Loc0\t\t\tLoc1\t\t\tLoc2");

  Serial.print("Target X: ");
  Serial.print(ld2450.getTargetX(0));
  Serial.print("\t\tTarget X: ");
  Serial.print(ld2450.getTargetX(1));
  Serial.print("\t\tTarget X: ");
  Serial.println(ld2450.getTargetX(2));

  Serial.print("Target Y: ");
  Serial.print(ld2450.getTargetY(0));
  Serial.print("\t\tTarget Y: ");
  Serial.print(ld2450.getTargetY(1));
  Serial.print("\t\tTarget Y: ");
  Serial.println(ld2450.getTargetY(2));

  Serial.print("Speed: ");
  Serial.print(ld2450.getSpeed(0));
  Serial.print("\t\tSpeed: ");
  Serial.print(ld2450.getSpeed(1));
  Serial.print("\t\tSpeed: ");
  Serial.println(ld2450.getSpeed(2));

  Serial.print("tDisRes: ");
  Serial.print(ld2450.getDistanceResolution(0));
  Serial.print("\t\tDisRes: ");
  Serial.print(ld2450.getDistanceResolution(1));
  Serial.print("\t\tDisRes: ");
  Serial.println(ld2450.getDistanceResolution(2));

  Serial.println("-----------------------------------------------");


}

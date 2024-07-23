int intValue;
char charValue;
float distance;

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    ;
  }

  Serial.println("Arduino is ready");
}

void loop() {
  if (Serial.available() > 0) {
    String receivedString = Serial.readStringUntil('\n');  // Read until a newline character is introduced

    if (receivedString.length() > 0) {
      receivedString.trim();  // Trim whitespace at the end of the line
      // Check if the received string contains a comma as pi 5 parses the serial messages as(color_index_char[0],int[np.degrees(horizontal_angle)], distance_mm)
      int commaIndex = receivedString.indexOf(',');
      if (commaIndex != -1) {  //if there is not index of a comma at all no execution will occur thus this value would be addressed as garbage data (filtering)
        // Extract the substring before the comma (potential color or angle)
        String beforeComma = receivedString.substring(0, commaIndex);

        // Check if it starts with 'g' or 'r' to determine color
        if (beforeComma.startsWith("g") || beforeComma.startsWith("r")) {
          charValue = beforeComma.charAt(0);  // Get the color character ('g' or 'r')

          // Extract the angle part as a substring after the color
          String anglePart = beforeComma.substring(1);  // Skip the color character
          intValue = anglePart.toInt();                 // Convert angle part to integer
        }

        // Extract distance part after the comma
        String distancePart = receivedString.substring(commaIndex + 1);  // Get substring after comma
        distance = distancePart.toFloat();                               // Convert distance part to float

        // Print received data for debugging
        Serial.print("Received - Color: ");
        Serial.print(charValue);
        Serial.print(", Angle: ");
        Serial.print(intValue);
        Serial.print(", Distance: ");
        Serial.println(distance);

        switch (charValue) {
          case 'g':
            if (distance > 200) {
              Serial.print("GREEN detected and distance is more than 200 by: ");
              Serial.println(distance - 200);
              // Example: Move Towards it (adjust until angle recieved = 0) call a function for adjustment
              CNTR_ANG(intValue);

            } else {
              Serial.println("GREEN detected and distance is less than 200mm.");
              Serial.println("Turn inside immediately.");
              // Example: Turn Servo Inside
              Green_maneuver(intValue);
            }
            break;

          case 'r':
            if (distance > 200) {
              Serial.print("RED detected and distance is more than 200 by: ");
              Serial.println(distance - 200);
              // Example: Move Towards it (adjust until angle recieved = 0) call a function for adjustment
              CNTR_ANG(intValue);
            } else {
              Serial.println("RED detected and distance is less than 200mm.");
              Serial.println("Turn inside immediately.");
              Red_maneuver(intValue);
              // Example: Turn Servo Outside
            }
            break;

          default:
            Serial.println("Unknown color detected.");
            // Handle case where an unknown color is received
            break;
        }
      }
    }
  }
  
}


void CNTR_ANG(int value) {
  if (intValue != 0) {
    //myservo.write(servo_center + intValue)
  } else {
    Serial.print("CENTERRRRRRRRRRRRRRRRRRRRREEEEEEEEEEEEEEEEEEEED");
  }
}
void Green_maneuver(int value) {
}
void Red_maneuver(int value) {
}

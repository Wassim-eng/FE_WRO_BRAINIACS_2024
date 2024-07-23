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
    String receivedString = Serial.readStringUntil('\n'); // Read until a newline character is introduced
    
    if (receivedString.length() > 0) {
      receivedString.trim(); // Trim whitespace at the end of the line
      // Check if the received string contains a comma as pi 5 parses the serial messages as(color_index_char[0],int[np.degrees(horizontal_angle)], distance_mm)
      int commaIndex = receivedString.indexOf(',');
      if (commaIndex != -1) { //if there is not index of a comma at all no execution will occur thus this value would be addressed as garbage data (filtering)
        // Extract the substring before the comma (potential color or angle)
        String beforeComma = receivedString.substring(0, commaIndex);
        
        // Check if it starts with 'g' or 'r' to determine color
        if (beforeComma.startsWith("g") || beforeComma.startsWith("r")) {
          charValue = beforeComma.charAt(0);  // Get the color character ('g' or 'r')
          
          // Extract the angle part as a substring after the color
          String anglePart = beforeComma.substring(1);  // Skip the color character
          intValue = anglePart.toInt();  // Convert angle part to integer
          
          // Check if the color indicates a negative angle ('g-')
          if (beforeComma.startsWith("g-")) {
            intValue *= -1;  // switch negative to interpret negative values from a string
          }
        } else {
          // Handle invalid data or fallback (debugging section)
          return;
        }
        
        // Extract distance part after the comma
        String distancePart = receivedString.substring(commaIndex + 1);  // Get substring after comma
        distance = distancePart.toFloat();  // Convert distance part to float
        
        // Print received data for debugging
        Serial.print("Received - Color: ");
        Serial.print(charValue);
        Serial.print(", Angle: ");
        Serial.print(intValue);
        Serial.print(", Distance: ");
        Serial.println(distance);
                
      } else {
        // Handle invalid data or fallback
        return;
      }
    }
  }
}

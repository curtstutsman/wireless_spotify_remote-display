#include <WiFi.h>
#include <HTTPClient.h>
#include <base64.h>
#include <ArduinoJson.h>
#include <JPEGDecoder.h>

#define RX2_PIN 16
#define TX2_PIN 17

#define THRESHOLD 127
#define IMAGE_SIZE 64
uint8_t rgbArray[IMAGE_SIZE][IMAGE_SIZE][3];
const char* filename = "/image.jpg";

const char* ssid = "IllinoisNet_Guest";
const char* password = "";
String client_id = "91c006a425e84d69876247225ab39a03"; 
String client_secret = "a9e8d50a696043f4a34db3692752a020"; 

String accessToken = "";
const char* refresh_token = "AQDAuNwFjpob35Skk81jcmjvVUIbm3C9jC90YVG9y1gTTMYHHZm2w0FDaCfeVRIkgqu7NU53z_0ZjDOtZI52QfF6TwGqoDXAiiIACC7znVcVo0gC6IzEwxIXZZZUbhCFylA"; // Refresh token for getting new access token

String lastAlbumLink = "";
unsigned long previousMillis = 0;
const long interval = 1500;

struct PlaybackState {
  int volumePercent;
  bool isPlaying;
  bool supportsVolume;
  String albumLink;
};
PlaybackState currentState;


PlaybackState getPlaybackState();
PlaybackState waitForValidPlaybackState();
void togglePlayback();
void raiseVolume();
void lowerVolume();
void nextTrack();
void previousTrack();
String getAccessToken();
bool saveImageToSPIFFS(String imageUrl);
bool decodeImageFromSPIFFS();
void sendImageData();
void initializeRgbArray();

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);

    WiFi.mode(WIFI_STA);
    delay(1000);  // short delay
    Serial.println(WiFi.macAddress());
    
    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        return;
    }

    //Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Request Access Token
    while (accessToken == ""){
      accessToken = getAccessToken();
    }

    Serial.println(accessToken);
    
     // **Initial Fetch of Album Art**
    currentState = getPlaybackState();
    if (currentState.volumePercent != -1 && currentState.albumLink != "") {
        lastAlbumLink = currentState.albumLink;
        if (saveImageToSPIFFS(lastAlbumLink)) {
            initializeRgbArray();
            if (decodeImageFromSPIFFS()) {
                sendImageData();
            }
        }
    }
}


String getAccessToken() {
  if (WiFi.status() == WL_CONNECTED) {
    
    HTTPClient http;
    String credentials = client_id + ":" + client_secret;
    String base64Credentials = base64::encode(credentials);

    http.begin("https://accounts.spotify.com/api/token");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    http.addHeader("Authorization", "Basic " + base64Credentials);

    String requestBody = "grant_type=refresh_token&refresh_token=" + String(refresh_token);
    int httpResponseCode = http.POST(requestBody);

    if (httpResponseCode == 200) {
      String response = http.getString();
      Serial.println("Access Token Response: " + response);

      // Parse the access token from the response
      int tokenIndex = response.indexOf("\"access_token\":\"");
      if (tokenIndex != -1) {
        int tokenStart = tokenIndex + strlen("\"access_token\":\"");
        int tokenEnd = response.indexOf("\"", tokenStart);
        String accessToken = response.substring(tokenStart, tokenEnd);
        http.end();
        return accessToken;
      } else {
        Serial.println("Access token not found in response.");
      }
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
      Serial.println(http.getString());
    }

    http.end(); // Free resources
  } else {
    Serial.println("Wi-Fi Disconnected");
  }

  return "";
}


PlaybackState getPlaybackState() {
  
    PlaybackState state = {-1, false, false, ""}; // Default values
    
    if (WiFi.status() == WL_CONNECTED) {
      
        HTTPClient http;
        http.begin("https://api.spotify.com/v1/me/player");
        http.addHeader("Authorization", String("Bearer ") + accessToken);
        http.setTimeout(interval);
        
        // Make the GET request
        int httpResponseCode = http.GET();
        if (httpResponseCode == 200) {
          
            String payload = http.getString();
            // Parse the JSON response
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, payload);
            
            if (!error) {
                state.supportsVolume = doc["device"]["supports_volume"];
                state.volumePercent = doc["device"]["volume_percent"]; // Volume percentage
                state.isPlaying = doc["is_playing"]; // Playing state
                state.albumLink = doc["item"]["album"]["images"][2]["url"].as<String>();
            } 
        } 
        http.end();
    } 
    return state; // Return the playback state
}


PlaybackState waitForValidPlaybackState() {
    PlaybackState state;
    do {
        state = getPlaybackState();
        if (state.volumePercent == -1) {
            lastAlbumLink = "";
            initializeRgbArray();
            sendImageData();
            delay(1500);
        }
    } while (state.volumePercent == -1);
    return state;
}


void togglePlayback() {
    HTTPClient http;
    String url;
    
    if (currentState.isPlaying) {
        url = "https://api.spotify.com/v1/me/player/pause";
        http.begin(url);
    } 
    else {
        url = "https://api.spotify.com/v1/me/player/play";
        http.begin(url);
    }
    
    http.addHeader("Authorization", String("Bearer ") + accessToken);
    http.addHeader("Content-Length", "0");
    int httpResponseCode = http.PUT(""); 
    http.end();
}


void raiseVolume() {
    if (currentState.volumePercent < 100 && currentState.supportsVolume) {
      
        int newVolume = currentState.volumePercent + 5; // Increase volume by 5
        if (newVolume > 100) newVolume = 100; // Cap at 100
        
        HTTPClient http;
        String url = "https://api.spotify.com/v1/me/player/volume?volume_percent=" +
        String(newVolume);
        http.begin(url);
        
        // Add the authorization header
        http.addHeader("Authorization", String("Bearer ") + accessToken);
        http.addHeader("Content-Length", "0");
        // Make the PUT request
        int httpResponseCode = http.PUT(""); // Empty body for volume command
        
        if (httpResponseCode > 0) {
            Serial.print("Volume raised to: ");
            Serial.println(newVolume);
            Serial.print("Response Code: ");
            Serial.println(httpResponseCode); // Print HTTP return code
        } 
        else {
            Serial.print("Error on HTTP request: ");
            Serial.println(httpResponseCode);
        }
        // Close connection
        http.end();
    } 
    else {
        Serial.println("Volume is already at maximum.");
    }
}


void lowerVolume() {
    if (currentState.volumePercent > 0 && currentState.supportsVolume) {
      
        int newVolume = currentState.volumePercent - 5; // Decrease volume by 5
        if (newVolume < 0) newVolume = 0; // Cap at 0
        
        HTTPClient http;
        String url = "https://api.spotify.com/v1/me/player/volume?volume_percent=" +
        String(newVolume);
        http.begin(url);
        
        // Add the authorization header
        http.addHeader("Authorization", String("Bearer ") + accessToken);
        // Make the PUT request
        http.addHeader("Content-Length", "0");
        int httpResponseCode = http.PUT(""); // Empty body for volume command
        
        if (httpResponseCode > 0) {
            Serial.print("Volume lowered to: ");
            Serial.println(newVolume);
            Serial.print("Response Code: ");
            Serial.println(httpResponseCode); // Print HTTP return code
        } 
        else {
            Serial.print("Error on HTTP request: ");
            Serial.println(httpResponseCode);
        }
        // Close connection
        http.end();
    } 
    else {
        Serial.println("Volume is already at minimum.");
    }
}


void nextTrack() {
    HTTPClient http;
    
    // Send a request to skip to the next track
    http.begin("https://api.spotify.com/v1/me/player/next");
    // Add the authorization header
    http.addHeader("Authorization", String("Bearer ") + accessToken);
    http.addHeader("Content-Length", "0");
    // Make the POST request (next track command)
    int httpResponseCode = http.POST(""); // Empty body for next track command
    
    if (httpResponseCode > 0) {
      Serial.print("Skipped to next track. Response Code: ");
      Serial.println(httpResponseCode); // Print HTTP return code

      if (saveImageToSPIFFS(currentState.albumLink)){
        initializeRgbArray();
        decodeImageFromSPIFFS();
        sendImageData();  
        lastAlbumLink = currentState.albumLink;
      }
      
    } 
    else {
      Serial.print("Error on HTTP request: ");
      Serial.println(httpResponseCode);
    }
    // Close connection
    http.end();
}


void previousTrack() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    // Send a request to go to the previous track
    http.begin("https://api.spotify.com/v1/me/player/previous");
    // Add the authorization header
    http.addHeader("Authorization", String("Bearer ") + accessToken);
    http.addHeader("Content-Length", "0");
    // Make the POST request (previous track command)
    int httpResponseCode = http.POST(""); // Empty body for previous track command
    
    if (httpResponseCode > 0) {
      Serial.print("Skipped to previous track. Response Code: ");
      Serial.println(httpResponseCode); // Print HTTP return code

      if (saveImageToSPIFFS(currentState.albumLink)){
        
        initializeRgbArray();
        decodeImageFromSPIFFS();
        sendImageData();
        lastAlbumLink = currentState.albumLink;  
      }
      
    } 
    else {
      Serial.print("Error on HTTP request: ");
      Serial.println(httpResponseCode);
    }
    // Close connection
    http.end(); 
  } 
  else {
    Serial.println("WiFi not connected. Cannot go to previous track.");
  }
}

bool saveImageToSPIFFS(String imageUrl) {
    HTTPClient http;
    http.begin(imageUrl);
    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
        WiFiClient* stream = http.getStreamPtr();
        File file = SPIFFS.open(filename, FILE_WRITE);
        if (!file) {
            Serial.println("Failed to open file for writing.");
            return false;
        }

        uint8_t buffer[512];
        int bytesRead;
        while ((bytesRead = stream->readBytes(buffer, sizeof(buffer))) > 0) {
            file.write(buffer, bytesRead);
        }
        file.close();
        Serial.println("Image saved to SPIFFS successfully.");
        return true;
    } else {
        Serial.printf("Failed to download image, HTTP response code: %d\n", httpResponseCode);
    }
    http.end();
    return false;
}

void initializeRgbArray() {
    for (int y = 0; y < IMAGE_SIZE; y++) {
        for (int x = 0; x < IMAGE_SIZE; x++) {
            rgbArray[y][x][0] = 0;
            rgbArray[y][x][1] = 0;
            rgbArray[y][x][2] = 0;
        }
    }
}

bool decodeImageFromSPIFFS() {
    if (JpegDec.decodeFsFile(filename) == 0) {
        // Failed to decode JPEG file.
        return false;
    }

    uint16_t mcu_w = JpegDec.MCUWidth;
    uint16_t mcu_h = JpegDec.MCUHeight;
    uint16_t max_x = JpegDec.width;
    uint16_t max_y = JpegDec.height;

    // Initialize THRESHOLD if not defined
    #ifndef THRESHOLD
    #define THRESHOLD 15  // Adjust based on RGB565 max values
    #endif

    while (JpegDec.read()) {
        int x = JpegDec.MCUx * mcu_w;
        int y = JpegDec.MCUy * mcu_h;

        for (int j = 0; j < mcu_h; j++) {
            for (int i = 0; i < mcu_w; i++) {
                if (x + i < max_x && y + j < max_y) {
                    uint16_t color = JpegDec.pImage[j * mcu_w + i];

                    // Extract RGB components from RGB565 color
                    uint8_t r = (color >> 11) & 0x1F;  // 5 bits
                    uint8_t g = (color >> 5) & 0x3F;   // 6 bits
                    uint8_t b = color & 0x1F;          // 5 bits

                    // Apply threshold to get RGB332 format
                    rgbArray[y + j][x + i][0] = r >> 2;
                    rgbArray[y + j][x + i][1] = g >> 3;
                    rgbArray[y + j][x + i][2] = b >> 3;
                }
            }
        }
    }
    // Image decoded successfully.
    return true;
}


// Send Image Data in RGB332 Format
void sendImageData() {
    delay(300);
    uint8_t pixel = 0;
    for (int y = 0; y < IMAGE_SIZE; y++) {
        for (int x = 0; x < IMAGE_SIZE; x++) {
            pixel = (rgbArray[y][x][0] & 0x07)         // Red
                    | ((rgbArray[y][x][1] & 0x07) << 3) // Green
                    | ((rgbArray[y][x][2] & 0x03) << 6); // Blue

            Serial2.write(pixel);  // Send as binary data
        }
    }
}


void loop() {
  
    // Update currentState periodically
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        currentState = waitForValidPlaybackState();

        // Check for new album art
        if (currentState.albumLink != "" && currentState.albumLink != lastAlbumLink) {
            if (saveImageToSPIFFS(currentState.albumLink)) {
                initializeRgbArray();
                if (decodeImageFromSPIFFS()) {
                    sendImageData();
                    lastAlbumLink = currentState.albumLink;
                }
            }
        }
    }

    // Handle serial input
    if (Serial2.available() > 0) {
        String input = Serial2.readStringUntil('\n'); // Read input until newline character
        input.trim();

        if (input.equalsIgnoreCase("info")) {
            Serial.println("Received 'info' request. Fetching current info...");

            if (currentState.volumePercent != -1) {
                Serial.print("Volume Percent: ");
                Serial.println(currentState.volumePercent);
                Serial.print("Is Playing: ");
                Serial.println(currentState.isPlaying ? "Yes" : "No");
                Serial.print("Volume Supported: ");
                Serial.println(currentState.supportsVolume ? "Yes" : "No");
                Serial.print("Image Link: ");
                Serial.println(currentState.albumLink);
            } else {
                Serial.println("Playback state is empty or invalid.");
            }
        }

        else if (input.equalsIgnoreCase("toggleplay")) {
            togglePlayback();
        }

        else if (input.equalsIgnoreCase("raiseVolume")) {
            raiseVolume();
        }

        else if (input.equalsIgnoreCase("lowerVolume")) {
            lowerVolume();
        }

        else if (input.equalsIgnoreCase("nextSong")) {
            nextTrack();
        }

        else if (input.equalsIgnoreCase("previousSong")) {
            previousTrack();
        }

        else {
            Serial.println("Unknown command: " + input);
        }
    }
}

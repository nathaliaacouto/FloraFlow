/**
 * Created by K. Suwatchai (Mobizt)
 *
 * Email: k_suwatchai@hotmail.com
 *
 * Github: https://github.com/mobizt/Firebase-ESP8266
 *
 * Copyright (c) 2023 mobizt
 *
 */

/** This example will show how to authenticate using
 * the legacy token or database secret with the new APIs (using config and auth data).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <SoftwareSerial.h>
// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

//Pinos de comunicacao serial com a ST Núcleo
#define Pin_ST_NUCLEO_RX    26  //Pino D1 da placa Node MCU
#define Pin_ST_NUCLEO_TX    27  //Pino D2 da placa Node MCU
SoftwareSerial SSerial(Pin_ST_NUCLEO_RX, Pin_ST_NUCLEO_TX);

/* 1. Define the WiFi credentials */
#define WIFI_SSID "uaifai-apolo"
#define WIFI_PASSWORD "bemvindoaocesar"

/* 2. If work with RTDB, define the RTDB URL and database secret */
#define DATABASE_URL "https://floraflow-2f7b7-default-rtdb.firebaseio.com/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define DATABASE_SECRET "j6YPsn9ZItoDWhejJpMujQNfgfwyr6Ycfrk0JllP"

/* 3. Define the Firebase Data object */
FirebaseData fbdo;

/* 4, Define the FirebaseAuth data for authentication data */
FirebaseAuth auth;

/* Define the FirebaseConfig data for config data */
FirebaseConfig config;

unsigned long dataMillis = 0;
int count = 0;
String receivedString = "";


void setup()
{

    Serial.begin(115200);
    SSerial.begin(115200);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the certificate file (optional) */
    // config.cert.file = "/cert.cer";
    // config.cert.file_storage = StorageType::FLASH;

    /* Assign the database URL and database secret(required) */
    config.database_url = DATABASE_URL;
    config.signer.tokens.legacy_token = DATABASE_SECRET;

    // Comment or pass false value when WiFi reconnection will control by your code or third party library e.g. WiFiManager
    Firebase.reconnectNetwork(true);

    // Since v4.4.x, BearSSL engine was used, the SSL buffer need to be set.
    // Large data transmission may require larger RX buffer, otherwise connection issue or data read time out can be occurred.
    fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);

    /* Initialize the library with the Firebase authen and config */
    Firebase.begin(&config, &auth);

    // Or use legacy authenticate method
    // Firebase.begin(DATABASE_URL, DATABASE_SECRET);
}

void loop()
{
    if (millis() - dataMillis > 5000)
    {
        dataMillis = millis();
        Serial.printf("Testing sending int... %s\n", Firebase.setInt(fbdo, "/test/int", count++) ? "ok" : fbdo.errorReason().c_str());
    }

    while (SSerial.available()) {
        // Read the incoming byte
        char received = SSerial.read();
        
        // Write the byte to the Serial monitor
        Serial.write(received);
        
        // Append the received character to the buffer
        receivedString += received;
        
        // Check if the end of the message is reached
        // For example, if we expect a newline character to signify the end
        if (received == '\n') {
            // Trim the newline character if necessary
            receivedString.trim();
            
            // Send the accumulated string to Firebase
            String path = "/planta/planta_escolhida";
            Serial.print("Sending to Firebase: ");
            Serial.println(receivedString);
            bool result = Firebase.setString(fbdo, path, receivedString);
            Firebase.push(fbdo, "log", receivedString);
            
            // Print the result of setting the string
            Serial.printf("Set String... %s\n", result ? "ok" : fbdo.errorReason().c_str());
            
            // Clear the buffer for the next message
            receivedString = "";
        }
    }
    delay(1);
}
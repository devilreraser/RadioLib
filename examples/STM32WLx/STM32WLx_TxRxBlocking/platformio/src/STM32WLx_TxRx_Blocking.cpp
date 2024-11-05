/*
  RadioLib STM32WLx Blocking Transmit/Receive Example

  This example transmits packets using STM32WL MCU with integrated
  (SX126x) LoRa radio.

  To successfully receive data, the following settings have to be the same
  on both transmitter and receiver:
  - carrier frequency
  - bandwidth
  - spreading factor
  - coding rate
  - sync word
  - preamble length
   
  Each packet contains up to 256 bytes of data, in the form of:
  - Arduino String
  - null-terminated char array (C-string)
  - arbitrary binary data (byte array)
  
  This example assumes Nucleo WL55JC1 is used. For other Nucleo boards
  or standalone STM32WL, some configuration such as TCXO voltage and
  RF switch control may have to be adjusted.

  Using blocking transmit is not recommended, as it will lead
  to inefficient use of processor time!
  Instead, interrupt transmit is recommended.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

#include <Arduino.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#define BUFFER_SIZE 128
#define QUEUE_LENGTH 32

static QueueHandle_t serialQueue = NULL;


// no need to configure pins, signals are routed to the radio internally
STM32WLx radio = new STM32WLx_Module();

#ifdef LORA_E5_DEV_BOARD

// set RF switch configuration for Nucleo WL55JC1
// NOTE: other boards may be different!
//       Some boards may not have either LP or HP.
//       For those, do not set the LP/HP entry in the table.
static const uint32_t rfswitch_pins[] =
                         {PC3,  PC4,  PC5, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},
    {STM32WLx::MODE_RX,    {HIGH, HIGH, LOW}},
    {STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH}},
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}},
    END_OF_MODE_TABLE,
};
#endif

extern "C" int _write(int file, char *ptr, int len) {
    // Ensure the length does not exceed BUFFER_SIZE
    int copyLen = (len < BUFFER_SIZE) ? len : BUFFER_SIZE;

    // Allocate memory for the message
    char *buffer = (char *)pvPortMalloc(copyLen + 1);
    if (buffer == NULL) {
        // Handle memory allocation failure
        return -1;
    }

    // Copy data and null-terminate
    memcpy(buffer, ptr, copyLen);
    buffer[copyLen] = '\0';

    if (serialQueue == NULL) 
    {
        vPortFree(buffer);
        return -1;
    }

    // Enqueue the buffer
    if (xQueueSend(serialQueue, &buffer, portMAX_DELAY) != pdPASS) {
        // Handle queue send failure
        vPortFree(buffer);
        return -1;
    }

    return len;  // Return the number of characters written
}

void serialTask(void *pvParameters) {
    char *msg;

    Serial.begin(115200);
    while (!Serial);      // Wait for Serial to initialize
    Serial.println("UART Initialized.");


    Serial.print("serialTask entered\r\n");

    // Create the queue
    serialQueue = xQueueCreate(QUEUE_LENGTH, sizeof(char *));
    if (serialQueue == NULL) {
        vTaskDelete(NULL);
        return;
    }

    Serial.print("serialTask queue created\r\n");
    printf("printf through queue Test Message\r\n");

    for (;;) {
        // Wait for a message to be available
        if (xQueueReceive(serialQueue, &msg, portMAX_DELAY) == pdPASS) {
            // Output the message
            Serial.print(msg);

            // Free the allocated memory
            vPortFree(msg);
        }

    }
}

void blinkTask(void *pvParameters) {
    const int ledPin = LED_BUILTIN;

    pinMode(ledPin, OUTPUT);

    while (1) {
        digitalWrite(ledPin, HIGH);
        printf("LED On\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));

        digitalWrite(ledPin, LOW);
        printf("LED Off\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void loraInit(void) {
    #ifdef LORA_E5_DEV_BOARD
    // set RF switch control configuration
    // this has to be done prior to calling begin()
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    #endif

    // initialize STM32WL with default settings, except frequency
    printf("[STM32WL] Initializing ... \r\n");
    int state = radio.begin(868.0);
    if (state == RADIOLIB_ERR_NONE) {
        printf("[STM32WL] Initializing success!\r\n");
    } else {
        printf("[STM32WL] Initializing failed, code %d\r\n",state);
        while (true) { delay(10); }
    }

    // set appropriate TCXO voltage for Nucleo WL55JC1
    state = radio.setTCXO(1.7);
    if (state == RADIOLIB_ERR_NONE) {
        printf("[STM32WL] setTCXO success!\r\n");
    } else {
        printf("[STM32WL] setTCXO failed, code %d\r\n",state);
        while (true) { delay(10); }
    }

}

void loraSendBlocking(int count) {
    printf("[STM32WL] Transmitting packet ... \r\n");

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    /*
    String str = "Hello World! #" + String(count);
    int state = radio.transmit(str);
    */

    // you can also transmit byte array up to 256 bytes long
    byte byteArr[] = {0x01, 0x23, 0x45, 0x56, 0x78, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);

    if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        printf("[STM32WL] Transmitting packet success!\r\n");

        // print measured data rate
        //printf("[STM32WL] Datarate:%d bps\r\n", (int)radio.getDataRate());
        printf("[STM32WL] Datarate:%.2f bps\r\n", radio.getDataRate());     /* needs build_flags =  -Wl,-u,_printf_float */

    } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
        // the supplied packet was longer than 256 bytes
        printf("[STM32WL] Transmitting packet too long!\r\n");

    } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
        // timeout occured while transmitting packet
        printf("[STM32WL] Transmitting packet timeout!\r\n");

    } else {
        // some other error occurred
        printf("[STM32WL] Transmitting packet failed, code %d\r\n",state);

    }

}

void loraRecvBlocking(void) {
    printf("[STM32WL] Waiting for incoming transmission ... \r\n");

    // you can receive data as an Arduino String
    /*
    String str;
    int state = radio.receive(str);
    */

    // you can also receive data as byte array
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);

    if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        printf("[STM32WL] Receiving packet success!\r\n");

        // print the data of the packet
        //printf("[STM32WL] Receiving packet Data:\t\t%s\r\n",str);
        printf("[STM32WL] Receiving packet Data:\t\t%02X %02X %02X %02X %02X %02X %02X %02X\r\n", byteArr[0], byteArr[1], byteArr[2], byteArr[3], byteArr[4], byteArr[5], byteArr[6], byteArr[7]);

        // print the RSSI (Received Signal Strength Indicator)
        // of the last received packet
        //printf("[STM32WL] Receiving packet RSSI:\t\t%d dBm\r\n", (int)radio.getRSSI());
        printf("[STM32WL] Receiving packet RSSI:\t\t%.2f dBm\r\n", radio.getRSSI());

        // print the SNR (Signal-to-Noise Ratio)
        // of the last received packet
        //printf("[STM32WL] Receiving packet SNR:\t\t%d dB\r\n", radio.getSNR());
        printf("[STM32WL] Receiving packet SNR: \t\t%.2f dB\r\n", radio.getSNR());

    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        // timeout occurred while waiting for a packet
        printf("[STM32WL] Receiving packet timeout!\r\n");

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        // packet was received, but is malformed
        printf("[STM32WL] Receiving packet CRC error!\r\n");

    } else {
        // some other error occurred
        printf("[STM32WL] Receiving packet failed, code %d\r\n",state);

    }

}

void loraTask(void *pvParameters) {
    loraInit();

    // counter to keep track of transmitted packets
    int count = 0;

    while(1) {
        loraSendBlocking(count++);
        loraRecvBlocking();
        // wait for a second before transmitting again
        //delay(1000);

    }
}

void setup() {
    
    // Create task for printf -> Serial handling
    xTaskCreate(serialTask,"SerialTask",1024, NULL, 1, NULL);

    // Create task for LED blinking handling
    xTaskCreate(blinkTask, "Blink Task", 128, NULL, 1, NULL);

    // Create task for LoRa Sending / Receiving
    xTaskCreate(loraTask,  "LoRa  Task", 256, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}





void loop() {
  // if rtos will be not executed
  printf("This is the loop\r\n");
  delay(1000);
}

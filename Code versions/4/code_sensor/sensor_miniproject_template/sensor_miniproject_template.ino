/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */


volatile int32_t lastInterruptTime = 0;

ISR(INT0_vect){
    // uint32_t interruptTime = millis();
    
    // if (interruptTime - lastInterruptTime > 100) {
    //     // Change PINE to PIND and PE4 to PD0 for Pin 21
    //     bool isPressed = (PIND & (1 << PD0)); 
        
    //     TState targetState = isPressed ? STATE_STOPPED : STATE_RUNNING;

    //     if (buttonState != targetState) {
    //         buttonState = targetState;
    //         stateChanged = true;
    //     }

    //     lastInterruptTime = interruptTime;
    // }
    uint32_t interruptTime = millis();
    
    if (interruptTime - lastInterruptTime > 100) {
        // Read PD0. With pull-down: if result > 0, button is being pressed.
        bool isPressed = (PIND & (1 << PD0)); 
        
        TState targetState = isPressed ? STATE_STOPPED : STATE_RUNNING;

        if (buttonState != targetState) {
            buttonState = targetState;
            stateChanged = true;
        }

        lastInterruptTime = interruptTime;
    }
}







// =============================================================
// Color sensor (TCS3200)
// =============================================================

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 *
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 *
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count * 10.
 *
 * Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 *
 *   static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
 *       // Set S2/S3 for each channel, measure edge count, multiply by 10
 *       *r = measureChannel(0, 0) * 10;  // red,   in Hz
 *       *g = measureChannel(1, 1) * 10;  // green, in Hz
 *       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 *   }
 */
 // ash: added on 13/3
#define S0_BIT 1  // Arduino 23 is PA1
#define S1_BIT 3  // Arduino 25 is PA3
#define S2_BIT 5  // Arduino 27 is PA5
#define S3_BIT 7  // Arduino 29 is PA7

#define OUT_BIT 2 // Arduino 47 is PL2 (Port L, Bit 2)

//  #define S0_PIN PA0
//  #define S1_PIN PA2
//  #define S2_PIN PA4
//  #define S3_PIN PA6
//  #define OUT_PIN PC7
 
 static void selectChannel(uint8_t s2, uint8_t s3) {
    // if (s2) {
    //     PORTA |= (1<<S2_PIN);
    // } else {
    //     PORTA &= ~(1<<S2_PIN);
    // }
    
    // if (s3) {
    //     PORTA |= (1<<S3_PIN);
    // } else {
    //     PORTA &= ~(1<<S3_PIN);
    // }
    // _delay_ms(20);

    if (s2) {
        PORTA |= (1 << S2_BIT);  // Changed from S2_PIN to S2_BIT
    } else {
        PORTA &= ~(1 << S2_BIT);
    }
    
    if (s3) {
        PORTA |= (1 << S3_BIT);  // Changed from S3_PIN to S3_BIT
    } else {
        PORTA &= ~(1 << S3_BIT);
    }
    _delay_ms(20);
}

static uint32_t measureChannel(uint8_t s2, uint8_t s3) {
    // selectChannel(s2, s3);

    // uint32_t count = 0;
    // volatile uint8_t prev = 0;

    // uint32_t start = millis();

    // while (millis() - start < 100) {
    //     volatile uint8_t curr = PINC & (1 << OUT_PIN);

    //     if (curr && !prev) {
    //         count++;
    //     }

    //     prev = curr;
    // }

    // return count;

    selectChannel(s2, s3);

    uint32_t count = 0;
    uint8_t prev = 0;
    uint32_t start = millis();

    // 100ms measurement window
    while (millis() - start < 100) {
        // Read Port L and mask for Bit 2 (Pin 47)
        uint8_t curr = (PINL & (1 << OUT_BIT)); 

        if (curr && !prev) {
            count++;
        }
        prev = curr;
    }

    return count;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    *r = measureChannel(0,0) * 10;
    *g = measureChannel(1,1) * 10;
    *b = measureChannel(0,1) * 10;
}

//for studio 16----------------------------------------------------
int currentSpeed = 200;

// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            {
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_STOPPED);
            break;

        // TODO (Activity 2): add COMMAND_COLOR case here.
        //   Call your color-reading function (which returns Hz), then send a
        //   response packet with the three channel frequencies in Hz.
        // ash: added on 15/3
        case COMMAND_COLOR:
            {
                uint32_t r,g,b;

                readColorChannels(&r,&g,&b);

                TPacket pkt;
                memset(&pkt,0,sizeof(pkt));

                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command = RESP_COLOR;

                pkt.params[0] = r;
                pkt.params[1] = g;
                pkt.params[2] = b;

                sendFrame(&pkt);
            }
            break;
        case COMMAND_BOT_FORWARD:
            {
                forward(currentSpeed);
                delay(1000);
                stop();
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_BOT_BACKWARD:
            {
                backward(currentSpeed);
                delay(1000);
                stop();
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_BOT_LEFT:
            {
                ccw(currentSpeed);
                delay(1000);
                stop();
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_BOT_RIGHT:
            {
                cw(currentSpeed);
                delay(1000);
                stop();
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_CHANGE_MOTOR_SPEED_DECREASE:
            {
                currentSpeed -= 25;
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_CHANGE_MOTOR_SPEED_INCREASE:
            {
                currentSpeed += 25;
                sendResponse(RESP_OK, 0);
            }
            break;
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // --- Serial Initialisation ---
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif

    // =============================================================
    // Activity 1: E-Stop Button Configuration (INT5 / PE5)
    // =============================================================
    cli();  
    // 1. Set Pin 21 (PD0) as an Input
    DDRD &= ~(1 << PD0);
    PORTD &= ~(1 << PD0);
    
    // 2. Configure the Trigger (EICRA for INT0-3)
    EICRA = (EICRA & ~((1 << ISC01) | (1 << ISC00))) | (1 << ISC00);

    // 3. Enable the Interrupt Mask for INT0
    EIMSK |= (1 << INT0);
    // =============================================================
    // Activity 2: Color Sensor (TCS3200)
    // =============================================================
    // Configure S0-S3 as outputs, OUT_PIN as input

    /*
    DDRA |= (1 << S0_PIN) | (1 << S1_PIN) | (1 << S2_PIN) | (1 << S3_PIN);
    DDRC &= ~(1 << PC7);

    // Set frequency scaling to 20% (S0=HIGH, S1=LOW)
    PORTA |= (1 << S0_PIN);
    PORTA &= ~(1 << S1_PIN);

    sei(); // Enable global interrupts

    */

   // Configure S0-S3 (Port A) as outputs
    DDRA |= (1 << S0_BIT) | (1 << S1_BIT) | (1 << S2_BIT) | (1 << S3_BIT);
    
    // Configure Pin 47 (Port L) as input
    DDRL &= ~(1 << OUT_BIT); 

    // Set frequency scaling to 20% (S0=HIGH, S1=LOW)
    PORTA |= (1 << S0_BIT);
    PORTA &= ~(1 << S1_BIT);

    sei();
}

void loop() {

    // // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);

    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}


#include "packets.h"
#include "serial_driver.h"
#include "arm_driver.h"

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
void handleArmCommand(char joint, int val);
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

volatile int32_t lastInterruptTime = 0;

ISR(INT0_vect){

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

 // ash: added on 13/3
#define S0_BIT 1  // Arduino 23 is PA1
#define S1_BIT 3  // Arduino 25 is PA3
#define S2_BIT 5  // Arduino 27 is PA5
#define S3_BIT 7  // Arduino 29 is PA7

#define OUT_BIT 0 // Arduino 49 is PL2 (Port L, Bit 2)

//  #define S0_PIN PA0
//  #define S1_PIN PA2
//  #define S2_PIN PA4
//  #define S3_PIN PA6
//  #define OUT_PIN PC7
 
 static void selectChannel(uint8_t s2, uint8_t s3) {

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


static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            stop();
            armStop()

            sei();
            {
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
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_BOT_BACKWARD:
            {
                backward(currentSpeed);
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_BOT_LEFT:
            {
                ccw(currentSpeed);
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_BOT_RIGHT:
            {
                cw(currentSpeed);
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_CHANGE_MOTOR_SPEED_DECREASE:
            {
                currentSpeed = min(currentSpeed + 25, 255);
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_CHANGE_MOTOR_SPEED_INCREASE:
            {
                currentSpeed = max(currentSpeed - 25, 0);
                sendResponse(RESP_OK, 0);
            }
            break;
        case COMMAND_STOP:{
            stop();
            sendResponse(RESP_OK, 0);
            } 
            break;
        case COMMAND_ARM: 
            {
            char joint = (char)cmd->params[0];
            int value = cmd->params[1];

            handleArmCommand(joint, value);

            sendResponse(RESP_OK, 0);
             break;
            }
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
    // E-Stop Button Configuration (INT5 / PE5)
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
    // Color Sensor (TCS3200)
    // =============================================================

   // Configure S0-S3 (Port A) as outputs
    DDRA |= (1 << S0_BIT) | (1 << S1_BIT) | (1 << S2_BIT) | (1 << S3_BIT);
    
    // Configure Pin 47 (Port L) as input
    DDRL &= ~(1 << OUT_BIT); 

    // Set frequency scaling to 20% (S0=HIGH, S1=LOW)
    PORTA |= (1 << S0_BIT);
    PORTA &= ~(1 << S1_BIT);

    // =============================================================
    // Servo Arm
    // =============================================================
    armInit()

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

#define ENCODER_1_A 15
#define ENCODER_1_B 14
#define ENCODER_2_A 17
#define ENCODER_2_B 16
#define ENCODER_3_A 19
#define ENCODER_3_B 18
#define JOYSTICK_X 22
#define JOYSTICK_Y 23
#define BUTTON_RESET 26
#define DEBUG_LOGS 0 // 1 for text logs, 0 for raw data packet log
#define LED 13

enum EncoderState
{
  STATE_LOCKED,
  STATE_TURN_RIGHT_START,
  STATE_TURN_RIGHT_MIDDLE,
  STATE_TURN_RIGHT_END,
  STATE_TURN_LEFT_START,
  STATE_TURN_LEFT_MIDDLE,
  STATE_TURN_LEFT_END,
  STATE_UNDECIDED
};

enum EncoderResult
{
  ENCODER_IDLE = 0,
  ENCODER_CW = 1,
  ENCODER_CCW = 2,
};

enum JoystickXResult
{
  JOYSTICK_Y_IDLE = 0,
  JOYSTICK_Y_NORTH = 1,
  JOYSTICK_Y_SOUTH = 2,
};

enum JoystickYResult
{
  JOYSTICK_X_IDLE = 0,
  JOYSTICK_X_EAST = 1,
  JOYSTICK_X_WEST = 2,
};

enum ButtonResult
{
  BUTTON_IDLE = 0,
  BUTTON_PRESSED = 1,
};

struct Encoder
{
  int pin_clk;
  int pin_dt;
  EncoderState state;
  uint8_t delta;
};

struct __attribute__((packed)) ColumnState
{
  union
  {
    struct
    {
      uint16_t encoder1 : 2;
      uint16_t encoder2 : 2;
      uint16_t encoder3 : 2;
      uint16_t joystick_x : 2;
      uint16_t joystick_y : 2;
      uint16_t button : 1;
      uint16_t padding : 5;
    };
    uint8_t data[2];
  };
};

struct __attribute__((packed)) TransmitState
{
  union
  {
    struct
    {
      ColumnState column;
      uint8_t crc8;
    };
    uint8_t data[3];
  };
};

Encoder Encoder1 = {.pin_clk = ENCODER_1_A, .pin_dt = ENCODER_1_B};
Encoder Encoder2 = {.pin_clk = ENCODER_2_A, .pin_dt = ENCODER_2_B};
Encoder Encoder3 = {.pin_clk = ENCODER_3_A, .pin_dt = ENCODER_3_B};

void setup()
{
  // Debug USB
  Serial.begin(115200);
  // RS485
  Serial1.begin(115200);
  // Reset button
  pinMode(BUTTON_RESET, INPUT);
  // Encoders
  EncoderInit(Encoder1);
  EncoderInit(Encoder2);
  EncoderInit(Encoder3);
  pinMode(LED, OUTPUT);
}

void EncoderInit(Encoder &encoder)
{
  pinMode(encoder.pin_clk, INPUT);
  pinMode(encoder.pin_dt, INPUT);
  encoder.state = STATE_LOCKED;
  encoder.delta = ENCODER_IDLE;
}

void loop()
{
  digitalWrite(LED, HIGH);
  bool doTransmit = false;
  bool hasSentReset = false;
  uint32_t millisLastTransmit = millis();
  uint32_t millisLastButtonPress = millis();

  TransmitState transmit;
  memset(&transmit, 0, sizeof(transmit));
  uint8_t button_reset = BUTTON_IDLE;
  int button_read_prev = 0;
  while (true)
  {
    uint8_t encoder1 = EncoderRead(Encoder1);
    uint8_t encoder2 = EncoderRead(Encoder2);
    uint8_t encoder3 = EncoderRead(Encoder3);

    uint8_t joystick_x = JoystickXRead();
    uint8_t joystick_y = JoystickYRead();

    // TODO: tidy up in functions
    // Evaluate reset button.
    int button_read = analogRead(BUTTON_RESET);
    // Read analog rather than digital value because the button is apparently leaking
    if ((button_read > 1000) && (button_read_prev <= 1000))
    {
      if (millis() - millisLastButtonPress >= 200)
      {
        button_reset = BUTTON_PRESSED;
        hasSentReset = false;
      }
      millisLastButtonPress = millis();
    }
    else
    {
      if (hasSentReset)
      {
        button_reset = BUTTON_IDLE;
      }
    }
    button_read_prev = button_read;

    // Evaluate Joystick
    if (joystick_x)
    {
      doTransmit = true;
      if (DEBUG_LOGS)
      {
        Serial.print("x: ");
        Serial.println(joystick_x);
      }
    }
    if (joystick_y)
    {
      doTransmit = true;
      if (DEBUG_LOGS)
      {
        Serial.print("y: ");
        Serial.println(joystick_y);
      }
    }

    // Evaluate button
    if (button_reset)
    {
      doTransmit = true;

      if (DEBUG_LOGS)
      {
        Serial.print("b: ");
        Serial.println(button_reset);
      }
    }

    // Evaluate encoders
    if (encoder1)
    {
      doTransmit = true;
      if (DEBUG_LOGS)
      {
        Serial.print("1: ");
        Serial.println(encoder1);
      }
    }
    if (encoder2)
    {
      doTransmit = true;
      if (DEBUG_LOGS)
      {
        Serial.print("2: ");
        Serial.println(encoder2);
      }
    }
    if (encoder3)
    {
      doTransmit = true;
      if (DEBUG_LOGS)
      {
        Serial.print("3: ");
        Serial.println(encoder3);
      }
    }

    if (doTransmit)
    {
      // Transmit max at every 20 ms
      if (millis() - millisLastTransmit >= 20)
      {
        transmit.column.joystick_x = joystick_x;
        transmit.column.joystick_y = joystick_y;
        transmit.column.button = button_reset;
        transmit.column.encoder1 = encoder1;
        transmit.column.encoder2 = encoder2;
        transmit.column.encoder3 = encoder3;
        transmit.crc8 = crc8(transmit.column.data, sizeof(transmit.column.data));

        millisLastTransmit = millis();
        doTransmit = false;
        hasSentReset = true;
        // Send data over RS485
        Serial1.write(transmit.data, sizeof(transmit));
        // Send data over USB debug port
        if (!DEBUG_LOGS)
        {
          Serial.write(transmit.data, sizeof(transmit));
        }
        memset(&transmit, 0, sizeof(transmit));
        digitalWrite(LED, !digitalRead(LED));
      }
    }
  }
}

uint8_t JoystickXRead(void)
{
  int joystick = analogRead(JOYSTICK_X);
  if (joystick < 400)
  {
    return JOYSTICK_X_WEST;
  }
  if (joystick > 700)
  {
    return JOYSTICK_X_EAST;
  }
  return JOYSTICK_X_IDLE;
}

uint8_t JoystickYRead(void)
{
  int joystick = analogRead(JOYSTICK_Y);
  if (joystick < 400)
  {
    return JOYSTICK_Y_SOUTH;
  }
  if (joystick > 700)
  {
    return JOYSTICK_Y_NORTH;
  }
  return JOYSTICK_Y_IDLE;
}

uint8_t EncoderRead(Encoder &encoder)
{
  encoder.delta = ENCODER_IDLE;
  bool a = !digitalRead(encoder.pin_clk);
  bool b = !digitalRead(encoder.pin_dt);
  switch (encoder.state)
  {
  case STATE_LOCKED:
    if (a && b)
    {
      encoder.state = STATE_UNDECIDED;
    }
    else if (!a && b)
    {
      encoder.state = STATE_TURN_LEFT_START;
    }
    else if (a && !b)
    {
      encoder.state = STATE_TURN_RIGHT_START;
    }
    else
    {
      encoder.state = STATE_LOCKED;
    };
    break;
  case STATE_TURN_RIGHT_START:
    if (a && b)
    {
      encoder.state = STATE_TURN_RIGHT_MIDDLE;
    }
    else if (!a && b)
    {
      encoder.state = STATE_TURN_RIGHT_END;
    }
    else if (a && !b)
    {
      encoder.state = STATE_TURN_RIGHT_START;
    }
    else
    {
      encoder.state = STATE_LOCKED;
    };
    break;
  case STATE_TURN_RIGHT_MIDDLE:
  case STATE_TURN_RIGHT_END:
    if (a && b)
    {
      encoder.state = STATE_TURN_RIGHT_MIDDLE;
    }
    else if (!a && b)
    {
      encoder.state = STATE_TURN_RIGHT_END;
    }
    else if (a && !b)
    {
      encoder.state = STATE_TURN_RIGHT_START;
    }
    else
    {
      encoder.state = STATE_LOCKED;
      encoder.delta = ENCODER_CW;
    };
    break;
  case STATE_TURN_LEFT_START:
    if (a && b)
    {
      encoder.state = STATE_TURN_LEFT_MIDDLE;
    }
    else if (!a && b)
    {
      encoder.state = STATE_TURN_LEFT_START;
    }
    else if (a && !b)
    {
      encoder.state = STATE_TURN_LEFT_END;
    }
    else
    {
      encoder.state = STATE_LOCKED;
    };
    break;
  case STATE_TURN_LEFT_MIDDLE:
  case STATE_TURN_LEFT_END:
    if (a && b)
    {
      encoder.state = STATE_TURN_LEFT_MIDDLE;
    }
    else if (!a && b)
    {
      encoder.state = STATE_TURN_LEFT_START;
    }
    else if (a && !b)
    {
      encoder.state = STATE_TURN_LEFT_END;
    }
    else
    {
      encoder.state = STATE_LOCKED;
      encoder.delta = ENCODER_CCW;
    };
    break;
  case STATE_UNDECIDED:
    if (a && b)
    {
      encoder.state = STATE_UNDECIDED;
    }
    else if (!a && b)
    {
      encoder.state = STATE_TURN_RIGHT_END;
    }
    else if (a && !b)
    {
      encoder.state = STATE_TURN_LEFT_END;
    }
    else
    {
      encoder.state = STATE_LOCKED;
    };
    break;
  }
  return encoder.delta;
}

uint8_t crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0x00;
  while (len--)
  {
    uint8_t extract = *data++;
    for (uint8_t temp = 8; temp; temp--)
    {
      uint8_t sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum)
      {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

/* Demo code for MPU6050 DMP
 * I thank Ian Hua.
 * Copyright (c) 2015 Match
 *
 * THE PROGRAM IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE PROGRAM OR THE USE OR OTHER DEALINGS IN
 * THE PROGRAM.
 */

// Define Necessary.
//#define OUTPUT_QUATERNION
//#define OUTPUT_EULER
//#define OUTPUT_ROLL_PITCH_YAW
//#define OUTPUT_FOR_TEAPOT
//#define OUTPUT_TEMPERATURE
#include "MPU6050_6Axis_MotionApps20.h"
#include "config.h"
#include "mbed.h"
#include <stdio.h>

#include "BLE.h"
#include "UARTService.h"

#define DEG_TO_RAD(x) (x * 0.01745329)
#define RAD_TO_DEG(x) (x * 57.29578)

// RawSerial pc(p21, p22);
//  MPU6050 mpu(I2C_SDA0, I2C_SCL0);     // sda, scl pin
MPU6050 mpu(p9, p10);
DigitalOut myled(p17);
DigitalOut myled2(p19);

const int FIFO_BUFFER_SIZE = 128;
uint8_t fifoBuffer[FIFO_BUFFER_SIZE];
uint16_t fifoCount;
uint16_t packetSize;
bool dmpReady;
uint8_t mpuIntStatus;
const int snprintf_buffer_size = 100;
char snprintf_buffer[snprintf_buffer_size];
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0,    0,    0,    0,
                            0,   0,    0, 0x00, 0x00, '\r', '\n'};

const char *LBRACKET = "[";
const char *RBRACKET = "]";
const char *COMMA = ",";
const char *BLANK = " ";
const char *PERIOD = ".";

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150; // empirical, to hold sampling to 200 Hz
const int NFast = 1000;   // the bigger, the better (but slower)
const int NSlow = 10000;  // ..
const int LinesBetweenHeaders = 5;
int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;

#define CONN_INTERVAL                                                          \
  25 /**< connection interval 20ms; in multiples of 0.125ms. (durationInMillis \
      * 1000) / UNIT_0_625_MS; */
#define CONN_SUP_TIMEOUT                                                       \
  8000 /**< Connection supervisory timeout (6 seconds); in multiples of        \
          0.125ms. */
#define SLAVE_LATENCY 0
//#define TICKER_INTERVAL   2.0f

BLE ble;
UARTService *uart;

static const char DEVICENAME[] = "Calibrate";
static volatile bool triggerSensorPolling = false;

static const char QUOTERNION_DESC[] = "quoternion";

static const uint8_t MPU6050_service_uuid[] = {
    0x45, 0x35, 0x56, 0x80, 0x0F, 0xD8, 0x5F, 0xB5,
    0x51, 0x48, 0x30, 0x27, 0x06, 0x9B, 0x3F, 0xD9};

static const uint8_t MPU6050_dmp_Characteristic_uuid[] = {
    0x45, 0x35, 0x56, 0x81, 0x0F, 0xD8, 0x5F, 0xB5,
    0x51, 0x48, 0x30, 0x27, 0x06, 0x9B, 0x3F, 0xD9};

uint8_t quoternionPayload[sizeof(float) * 4] = {
    0,
};

void sendString(const char *str) { uart->writeString(str); }

void sendFloat(float num) {
  const int snprintf_buffer_size = 100;
  char snprintf_buffer[snprintf_buffer_size];
  snprintf(snprintf_buffer, snprintf_buffer_size, "%6.2f\n", num); // right
  sendString(snprintf_buffer);
}

void sendInt(const int num) {
  char str[12];
  sprintf(str, "%d", num);
  uart->writeString(str);
}

void sendResult(int result) {
  const int snprintf_buffer_size = 100;
  char snprintf_buffer[snprintf_buffer_size];

  snprintf(snprintf_buffer, snprintf_buffer_size, "%d\n", result);
  sendString(snprintf_buffer);
}

bool Init() {

  mpu.initialize();
  if (mpu.testConnection()) {
  } else {
    return false;
  }
  if (mpu.dmpInitialize() == 0) {
  } else {
    return false;
  }
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffsetUser(0);
  mpu.setYGyroOffsetUser(0);
  mpu.setZGyroOffsetUser(0);

  myled = 1;
  return true;
}

void disconnectionCallback(
    const Gap::DisconnectionCallbackParams_t *params) // Mod
{

  // DEBUG("Disconnected handle %u, reason %u\n", handle, reason);
  // DEBUG("Restarting the advertising process\n\r");

  ble.startAdvertising();
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params) {

  Gap::ConnectionParams_t gap_conn_params;
  gap_conn_params.minConnectionInterval =
      Gap::MSEC_TO_GAP_DURATION_UNITS(CONN_INTERVAL);
  gap_conn_params.maxConnectionInterval =
      Gap::MSEC_TO_GAP_DURATION_UNITS(CONN_INTERVAL);
  gap_conn_params.connectionSupervisionTimeout =
      Gap::MSEC_TO_GAP_DURATION_UNITS(CONN_SUP_TIMEOUT);
  gap_conn_params.slaveLatency = SLAVE_LATENCY;

  if (ble.updateConnectionParams(params->handle, &gap_conn_params) !=
      BLE_ERROR_NONE) {
    // DEBUG("failed to update connection paramter\r\n");
  }
}

void timeoutCallback(const Gap::TimeoutSource_t source) {
  // DEBUG("TimeOut\n\r");
  // DEBUG("Restarting the advertising process\n\r");

  ble.startAdvertising();
}

void ForceHeader() { LinesOut = 99; }

void GetSmoothed() {
  int16_t RawValue[6];
  int i;
  long Sums[6];
  for (i = iAx; i <= iGz; i++) {
    Sums[i] = 0;
  }
  //    unsigned long Start = micros();

  for (i = 1; i <= N; i++) {
    // get sums
    mpu.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
                   &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
    // if ((i % 500) == 0) {sendString(PERIOD);}
    wait_us(usDelay);
    for (int j = iAx; j <= iGz; j++) {
      // sendInt(RawValue[j]);
      Sums[j] = Sums[j] + RawValue[j];
    }
  } // get sums
    //    unsigned long usForN = micros() - Start;
    //    Serial.print(" reading at ");
    //    Serial.print(1000000/((usForN+N/2)/N));
    //    Serial.println(" Hz");
  for (i = iAx; i <= iGz; i++) {
    Smoothed[i] = (Sums[i] + N / 2) / N;
  }
} // GetSmoothed

void SetOffsets(int TheOffsets[6]) {
  mpu.setXAccelOffset(TheOffsets[iAx]);
  mpu.setYAccelOffset(TheOffsets[iAy]);
  mpu.setZAccelOffset(TheOffsets[iAz]);
  mpu.setXGyroOffsetUser(TheOffsets[iGx]);
  mpu.setYGyroOffsetUser(TheOffsets[iGy]);
  mpu.setZGyroOffsetUser(TheOffsets[iGz]);

} // SetOffsets

void ShowProgress() {

  char buffer[128];
  snprintf(buffer, sizeof(buffer), " x: %d\n y: %d\n z: %d\n xg: %d\n yg: %d\n zg: %d\n DONE\n", LowOffset[0], LowOffset[1],
  LowOffset[2], LowOffset[3], LowOffset[4],LowOffset[5]);
  char *num_string = buffer; // String terminator is added by snprintf

  uart->writeString(num_string);

} // ShowProgress

void SetAveraging(int NewN) {
  N = NewN;
  // sendString("averaging ");
  // sendInt(N);
  // sendString(" readings each time\n");
} // SetAveraging

void PullBracketsOut() {
  bool Done = false;
  int NextLowOffset[6];
  int NextHighOffset[6];

  // sendString("expanding:");
  ForceHeader();

  while (!Done) {
    Done = true;
    SetOffsets(LowOffset);
    GetSmoothed();
    for (int i = iAx; i <= iGz; i++) {
      // got low values
      LowValue[i] = Smoothed[i];
      if (LowValue[i] >= Target[i]) {
        Done = false;
        NextLowOffset[i] = LowOffset[i] - 1000;
      } else {
        NextLowOffset[i] = LowOffset[i];
      }
    } // got low values

    SetOffsets(HighOffset);
    GetSmoothed();
    for (int i = iAx; i <= iGz; i++) {
      // got high values
      HighValue[i] = Smoothed[i];
      if (HighValue[i] <= Target[i]) {
        Done = false;
        NextHighOffset[i] = HighOffset[i] + 1000;
      } else {
        NextHighOffset[i] = HighOffset[i];
      }
    } // got high values
    // ShowProgress();
    for (int i = iAx; i <= iGz; i++) {
      LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
      HighOffset[i] = NextHighOffset[i]; // ..
    }
  } // keep going
} // PullBracketsOut

void PullBracketsIn() {
  bool AllBracketsNarrow;
  bool StillWorking;
  int NewOffset[6];

  // sendString("\nclosing in:");
  AllBracketsNarrow = false;
  ForceHeader();
  StillWorking = true;
  while (StillWorking) {
    StillWorking = false;
    if (AllBracketsNarrow && (N == NFast)) {
      SetAveraging(NSlow);
    } else {
      AllBracketsNarrow = true; // tentative
    }
    for (int i = iAx; i <= iGz; i++) {
      if (HighOffset[i] <= (LowOffset[i] + 1)) {
        NewOffset[i] = LowOffset[i];
      } else {
        // binary search
        StillWorking = true;
        NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
        if (HighOffset[i] > (LowOffset[i] + 10)) {
          AllBracketsNarrow = false;
        }
      } // binary search
    }
    SetOffsets(NewOffset);
    GetSmoothed();
    for (int i = iAx; i <= iGz; i++) {
      // closing in
      if (Smoothed[i] > Target[i]) {
        // use lower half
        HighOffset[i] = NewOffset[i];
        HighValue[i] = Smoothed[i];
      } // use lower half
      else {
        // use upper half
        LowOffset[i] = NewOffset[i];
        LowValue[i] = Smoothed[i];
      } // use upper half
    }   // closing in
        // ShowProgress();
  }     // still working

} // PullBracketsIn

void my_analogin_init(void) {
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
  NRF_ADC->CONFIG =
      (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
}

uint16_t my_analogin_read_u16(void) {
  NRF_ADC->CONFIG &= ~ADC_CONFIG_PSEL_Msk;
  NRF_ADC->CONFIG |= ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos;
  NRF_ADC->TASKS_START = 1;
  while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) ==
         ADC_BUSY_BUSY_Busy) {
  };
  return (uint16_t)NRF_ADC->RESULT; // 10 bit
}

void sendVoltage() {
  // voltage
  const int snprintf_buffer_size = 100;
  char snprintf_buffer[snprintf_buffer_size];

  float value = (float)my_analogin_read_u16();
  value = (value * 3.6) / 1024.0;
  // value = (value - 2.9 + 0.5)*200;
  snprintf(snprintf_buffer, snprintf_buffer_size, "lv_%6.2f\n", value);
  sendString(snprintf_buffer);
}

void onDataWritten(const GattWriteCallbackParams *params) {
  if ((uart != NULL) && (params->handle == uart->getTXCharacteristicHandle())) {
    uint16_t bytesRead = params->len;

    if (bytesRead == 3) {

      sendString("calibrating...\n");

      myled2 = 1;
      myled = 0;

      for (int i = iAx; i <= iGz; i++) {
        // set targets and initial guesses
        Target[i] = 0; // must fix for ZAccel
        HighOffset[i] = 0;
        LowOffset[i] = 0;
      } // set targets and initial guesses

      Target[iAz] = 16384;
      SetAveraging(NFast);
      PullBracketsOut();
      PullBracketsIn();

      ShowProgress();

      myled2 = 0;
      myled = 1;
    }

    if (bytesRead == 4) {
      sendResult(200);
    }
  }
}

void bleInit() {
  ble.init();
  ble.onDisconnection(disconnectionCallback);
  ble.onConnection(connectionCallback);
  ble.onDataWritten(onDataWritten);

  /* setup advertising */
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                   (const uint8_t *)DEVICENAME,
                                   sizeof(DEVICENAME) - 1);
  ble.accumulateAdvertisingPayload(
      GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
      (const uint8_t *)UARTServiceUUID_reversed,
      sizeof(UARTServiceUUID_reversed));

  ble.setAdvertisingInterval(1000);
  ble.startAdvertising();
  uart = new UARTService(ble);

  // ble.addService(MPU6050DMPService);
}

int main() {
  MBED_ASSERT(Init() == true);
  bleInit();
  my_analogin_init();
  wait(3);
}

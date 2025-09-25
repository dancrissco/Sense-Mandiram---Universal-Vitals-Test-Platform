#include <Wire.h>
#include <LSM6DS3.h>                 // IMU library you're already using
#include <SilabsMicrophoneAnalog.h>  // Silicon Labs analog mic helper

// -------------------- IMU (LSM6DS3) --------------------
LSM6DS3 myIMU(I2C_MODE, 0x6A);  // per Seeed wiki

// -------------------- Microphone -----------------------
#define MIC_DATA_PIN  PC9
#define MIC_PWR_PIN   PC8
#define NUM_SAMPLES   128

MicrophoneAnalog mic(MIC_DATA_PIN, MIC_PWR_PIN);
uint32_t mic_buf[NUM_SAMPLES];
uint32_t mic_local[NUM_SAMPLES];
volatile bool mic_ready = false;

uint32_t last_frame_ms = 0;
float fs_est_hz = 8000.0f;  // dynamic estimate

// Smoothing for outgoing features
static const float EMA_ALPHA = 0.1f;
float sm_ac_rms = 0, sm_heart_rms = 0, sm_breath_rms = 0;
float sm_pitch   = 0, sm_roll      = 0;

static inline float rad2deg(float r) { return r * 57.2957795f; }
void on_mic_samples_ready() { memcpy(mic_local, mic_buf, NUM_SAMPLES * sizeof(uint32_t)); mic_ready = true; }

// -------------------- BLE (Silabs BGAPI) ----------------
// Ensure: Tools → Protocol stack → BLE (Silabs)

static void ble_initialize_gatt_db();
static void ble_start_advertising();

static const uint8_t advertised_name[] = "MG24-Ausc";

static uint16_t gattdb_session_id;
static uint16_t generic_access_service_handle;
static uint16_t name_characteristic_handle;

static uint16_t ausc_service_handle;
static uint16_t feat_char_handle;

static bool feat_notify_enabled = false;  // set when client enables notifications

// Use SDK-provided uuid_128 (already defined in sl_bgapi.h)
// Custom 128-bit UUIDs (little-endian)
static const uuid_128 AUSC_SVC_UUID  = { .data = { 0xfe, 0xca, 0xed, 0xfe, 0x00, 0x00, 0x10, 0x9a, 0x5c, 0x4b, 0x6d, 0x7e, 0x01, 0x00, 0xde, 0xc0 } };
static const uuid_128 FEAT_CHR_UUID  = { .data = { 0xfe, 0xca, 0xed, 0xfe, 0x00, 0x00, 0x10, 0x9a, 0x5c, 0x4b, 0x6d, 0x7e, 0x02, 0x00, 0xde, 0xc0 } };

// Packed 15-byte frame (little-endian): <Ihhhhhb
#pragma pack(push, 1)
struct Frame15 {
  uint32_t t_ms;  // millis
  int16_t ac;     // mic_ac_rms * 100
  int16_t heart;  // mic_heart_rms * 100
  int16_t breath; // mic_breath_rms * 100
  int16_t pitch;  // deg * 10
  int16_t roll;   // deg * 10
  int8_t  flags;  // future bitfield
};
#pragma pack(pop)

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Power gate for IMU (per Seeed wiki)
  pinMode(PD5, OUTPUT);
  digitalWrite(PD5, HIGH);
  delay(300);

  if (myIMU.begin() != 0) {
    Serial.println("# ERR: IMU begin failed");
    while (1) { delay(1000); }
  }

  // Mic init
  mic.begin(mic_buf, NUM_SAMPLES);
  mic.startSampling(on_mic_samples_ready);

  Serial.println("Silicon Labs BLE auscultation (BGAPI)");
}

void loop() {
  if (!mic_ready) return;
  mic_ready = false;

  // Estimate mic frame rate
  uint32_t now_ms = millis();
  if (last_frame_ms != 0) {
    float dt_s = (now_ms - last_frame_ms) / 1000.0f;
    if (dt_s > 0.0001f) {
      float fs_new = NUM_SAMPLES / dt_s;
      if (fs_new > 500 && fs_new < 48000) fs_est_hz = 0.9f * fs_est_hz + 0.1f * fs_new;
    }
  }
  last_frame_ms = now_ms;

  mic.stopSampling();

  // -------- Audio features --------
  double sum = 0.0;
  for (uint16_t i = 0; i < NUM_SAMPLES; i++) sum += mic_local[i];
  float mean = (float)(sum / NUM_SAMPLES);

  double ac_sumsq = 0.0;
  for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
    float x = (float)mic_local[i] - mean;
    ac_sumsq += x * x;
  }
  float mic_ac_rms = sqrtf((float)(ac_sumsq / NUM_SAMPLES));

  // Single-pole low-pass “bands”
  float dt = 1.0f / fs_est_hz;
  auto lp_alpha = [&](float fc_hz) {
    float rc = 1.0f / (6.2831853f * fc_hz);
    return dt / (rc + dt);
  };
  float a_heart  = constrain(lp_alpha(200.0f),  0.001f, 0.5f);
  float a_breath = constrain(lp_alpha(1000.0f), 0.001f, 0.5f);

  float yh = 0.0f, yb = 0.0f;
  double sumsq_h = 0.0, sumsq_b = 0.0;
  for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
    float x = (float)mic_local[i] - mean;
    yh += a_heart  * (x - yh);
    yb += a_breath * (x - yb);
    sumsq_h += yh * yh;
    sumsq_b += yb * yb;
  }
  float mic_heart_rms  = sqrtf((float)(sumsq_h / NUM_SAMPLES));
  float mic_breath_rms = sqrtf((float)(sumsq_b / NUM_SAMPLES));

  // -------- IMU tilt (accel only) --------
  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();

  static float ax_f = 0, ay_f = 0, az_f = 1;
  const float ALPHA_ACC = 0.1f;
  ax_f = (1 - ALPHA_ACC) * ax_f + ALPHA_ACC * ax;
  ay_f = (1 - ALPHA_ACC) * ay_f + ALPHA_ACC * ay;
  az_f = (1 - ALPHA_ACC) * az_f + ALPHA_ACC * az;

  float pitch_deg = rad2deg(atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)));
  float roll_deg  = rad2deg( atan2f( ay_f, az_f ) );

  // Smooth outgoing values
  sm_ac_rms     = (1-EMA_ALPHA)*sm_ac_rms     + EMA_ALPHA*mic_ac_rms;
  sm_heart_rms  = (1-EMA_ALPHA)*sm_heart_rms  + EMA_ALPHA*mic_heart_rms;
  sm_breath_rms = (1-EMA_ALPHA)*sm_breath_rms + EMA_ALPHA*mic_breath_rms;
  sm_pitch      = (1-EMA_ALPHA)*sm_pitch      + EMA_ALPHA*pitch_deg;
  sm_roll       = (1-EMA_ALPHA)*sm_roll       + EMA_ALPHA*roll_deg;

  // -------- Notify when client subscribed --------
  if (feat_notify_enabled) {
    Frame15 f;
    f.t_ms   = now_ms;
    f.ac     = (int16_t)lroundf(sm_ac_rms     * 100.0f);
    f.heart  = (int16_t)lroundf(sm_heart_rms  * 100.0f);
    f.breath = (int16_t)lroundf(sm_breath_rms * 100.0f);
    f.pitch  = (int16_t)lroundf(sm_pitch      * 10.0f);
    f.roll   = (int16_t)lroundf(sm_roll       * 10.0f);
    f.flags  = 0;

    sl_status_t sc = sl_bt_gatt_server_notify_all(feat_char_handle,
                                                  sizeof(f),
                                                  (const uint8_t*)&f);
    (void)sc; // optional check
  }

  mic.startSampling(on_mic_samples_ready);
}

// ---------------- BGAPI event handler -------------------
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      Serial.begin(115200);
      Serial.println("BLE stack booted");
      ble_initialize_gatt_db();
      ble_start_advertising();
      Serial.println("BLE advertisement started as 'MG24-Ausc'");
      break;

    case sl_bt_evt_connection_opened_id:
      Serial.println("BLE connection opened");
      break;

    case sl_bt_evt_connection_closed_id:
      Serial.println("BLE connection closed");
      ble_start_advertising();
      Serial.println("BLE advertisement restarted");
      feat_notify_enabled = false; // reset on disconnect
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
      if (evt->data.evt_gatt_server_characteristic_status.characteristic == feat_char_handle) {
        if (evt->data.evt_gatt_server_characteristic_status.client_config_flags & sl_bt_gatt_notification) {
          Serial.println("Features notify enabled");
          feat_notify_enabled = true;
        } else {
          Serial.println("Features notify disabled");
          feat_notify_enabled = false;
        }
      }
      break;

    default:
      break;
  }
}

// ---------------- Advertising / GATT DB -----------------
static void ble_start_advertising()
{
  static uint8_t advertising_set_handle = 0xff;
  static bool init = true;
  sl_status_t sc;

  if (init) {
    sc = sl_bt_advertiser_create_set(&advertising_set_handle);
    app_assert_status(sc);

    // 100 ms (0.625 ms units → 160)
    sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, 160, 0, 0);
    app_assert_status(sc);

    init = false;
  }

  sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                             sl_bt_advertiser_general_discoverable);
  app_assert_status(sc);

  sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                     sl_bt_advertiser_connectable_scannable);
  app_assert_status(sc);
}

static void ble_initialize_gatt_db()
{
  sl_status_t sc;

  // New GATT DB session
  sc = sl_bt_gattdb_new_session(&gattdb_session_id);
  app_assert_status(sc);

  // ----- Generic Access (Device Name) -----
  const uint8_t generic_access_service_uuid[] = { 0x00, 0x18 };
  sc = sl_bt_gattdb_add_service(gattdb_session_id,
                                sl_bt_gattdb_primary_service,
                                SL_BT_GATTDB_ADVERTISED_SERVICE,
                                sizeof(generic_access_service_uuid),
                                generic_access_service_uuid,
                                &generic_access_service_handle);
  app_assert_status(sc);

  const sl_bt_uuid_16_t device_name_characteristic_uuid = { .data = { 0x00, 0x2A } };
  sc = sl_bt_gattdb_add_uuid16_characteristic(gattdb_session_id,
                                              generic_access_service_handle,
                                              SL_BT_GATTDB_CHARACTERISTIC_READ,
                                              0x00, 0x00,
                                              device_name_characteristic_uuid,
                                              sl_bt_gattdb_fixed_length_value,
                                              sizeof(advertised_name) - 1,
                                              sizeof(advertised_name) - 1,
                                              advertised_name,
                                              &name_characteristic_handle);
  app_assert_status(sc);

  sc = sl_bt_gattdb_start_service(gattdb_session_id, generic_access_service_handle);
  app_assert_status(sc);

  // ----- Our custom “Auscultation” primary service -----
  sc = sl_bt_gattdb_add_service(gattdb_session_id,
                                sl_bt_gattdb_primary_service,
                                SL_BT_GATTDB_ADVERTISED_SERVICE,
                                sizeof(AUSC_SVC_UUID),
                                AUSC_SVC_UUID.data,
                                &ausc_service_handle);
  app_assert_status(sc);

  // Fixed-length characteristic (15 bytes), READ | NOTIFY
  uint8_t zeros[15] = {0};
  sc = sl_bt_gattdb_add_uuid128_characteristic(gattdb_session_id,
                                               ausc_service_handle,
                                               SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,
                                               0x00, 0x00,                 // descriptor flags, keys
                                               FEAT_CHR_UUID,
                                               sl_bt_gattdb_fixed_length_value,
                                               sizeof(zeros), sizeof(zeros),
                                               zeros,
                                               &feat_char_handle);
  app_assert_status(sc);

  // Start our service & commit DB
  sc = sl_bt_gattdb_start_service(gattdb_session_id, ausc_service_handle);
  app_assert_status(sc);

  sc = sl_bt_gattdb_commit(gattdb_session_id);
  app_assert_status(sc);
}

#ifndef ARDUINO_SILABS_STACK_BLE_SILABS
  #error "This sketch requires Tools > Protocol stack > 'BLE (Silabs)'."
#endif

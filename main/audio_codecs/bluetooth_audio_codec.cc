#include "bluetooth_audio_codec.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "BT_AUDIO";

// 全局实例指针，用于回调
static BluetoothAudioCodec* g_bt_audio_instance = nullptr;

BluetoothAudioCodec::BluetoothAudioCodec()
    : initialized_(false), connected_(false), volume_(50), muted_(false) {
    g_bt_audio_instance = this;
}

BluetoothAudioCodec::~BluetoothAudioCodec() {
    Stop();
    g_bt_audio_instance = nullptr;
}

bool BluetoothAudioCodec::Initialize() {
    ESP_LOGI(TAG, "Initializing Bluetooth Audio Codec");
    
    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BT controller");
        return false;
    }
    
    if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable BT controller");
        return false;
    }
    
    // 初始化蓝牙协议栈
    if (esp_bluedroid_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bluedroid");
        return false;
    }
    
    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable bluedroid");
        return false;
    }
    
    // 设置设备名称
    esp_bt_dev_set_device_name("ESP32S3_Audio");
    
    // 注册GAP回调
    esp_bt_gap_register_callback(BtAppGapCallback);
    
    // 设置I/O能力和PIN码
    esp_bt_io_cap_t io_cap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &io_cap, sizeof(esp_bt_io_cap_t));
    
    esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);
    
    // 初始化A2DP接收器
    if (esp_a2d_sink_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize A2DP sink");
        return false;
    }
    
    // 注册A2DP回调
    if (esp_a2d_register_callback(BtAppA2dCallback) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register A2DP callback");
        return false;
    }
    
    // 注册A2DP数据回调
    if (esp_a2d_sink_register_data_callback(BtAppA2dDataCallback) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register A2DP data callback");
        return false;
    }
    
    // 初始化AVRCP控制器
    if (esp_avrc_ct_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AVRC controller");
        return false;
    }
    
    // 注册AVRCP回调
    if (esp_avrc_ct_register_callback(BtAppAvrcCallback) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register AVRC callback");
        return false;
    }
    
    initialized_ = true;
    return true;
}

bool BluetoothAudioCodec::Start() {
    if (!initialized_) {
        ESP_LOGE(TAG, "Bluetooth Audio Codec not initialized");
        return false;
    }
    
    // 设置可发现模式
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    
    ESP_LOGI(TAG, "Bluetooth Audio Codec started, waiting for connections");
    return true;
}

bool BluetoothAudioCodec::Stop() {
    if (!initialized_) {
        return true;
    }
    
    // 断开连接
    if (connected_) {
        Disconnect();
    }
    
    // 停止AVRCP
    esp_avrc_ct_deinit();
    
    // 停止A2DP
    esp_a2d_sink_deinit();
    
    // 停止蓝牙协议栈
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    
    // 停止蓝牙控制器
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    
    initialized_ = false;
    return true;
}

bool BluetoothAudioCodec::Pause() {
    if (!connected_) {
        return false;
    }
    
    // 发送AVRCP暂停命令
    esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PAUSE, ESP_AVRC_PT_CMD_STATE_PRESSED);
    esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PAUSE, ESP_AVRC_PT_CMD_STATE_RELEASED);
    return true;
}

bool BluetoothAudioCodec::Resume() {
    if (!connected_) {
        return false;
    }
    
    // 发送AVRCP播放命令
    esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_PRESSED);
    esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_RELEASED);
    return true;
}

bool BluetoothAudioCodec::SetVolume(int volume) {
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    
    volume_ = volume;
    
    if (connected_) {
        // 发送AVRCP音量命令
        uint8_t vol = (uint8_t)((volume * 127) / 100);
        esp_avrc_ct_send_set_absolute_volume_cmd(0, vol);
    }
    
    return true;
}

bool BluetoothAudioCodec::SetMute(bool mute) {
    muted_ = mute;
    return true;
}

bool BluetoothAudioCodec::IsReady() const {
    return initialized_ && connected_;
}

bool BluetoothAudioCodec::StartDiscovery() {
    if (!initialized_) {
        return false;
    }
    
    ESP_LOGI(TAG, "Starting device discovery");
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    return true;
}

bool BluetoothAudioCodec::StopDiscovery() {
    if (!initialized_) {
        return false;
    }
    
    ESP_LOGI(TAG, "Stopping device discovery");
    esp_bt_gap_cancel_discovery();
    return true;
}

bool BluetoothAudioCodec::Connect(const std::string& device_address) {
    if (!initialized_) {
        return false;
    }
    
    ESP_LOGI(TAG, "Connecting to device: %s", device_address.c_str());
    
    esp_bd_addr_t addr;
    sscanf(device_address.c_str(), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
           &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]);
    
    esp_a2d_sink_connect(addr);
    return true;
}

bool BluetoothAudioCodec::Disconnect() {
    if (!initialized_ || !connected_) {
        return false;
    }
    
    ESP_LOGI(TAG, "Disconnecting from device");
    esp_a2d_sink_disconnect();
    connected_ = false;
    return true;
}

// 静态回调函数
void BluetoothAudioCodec::BtAppA2dCallback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    if (g_bt_audio_instance == nullptr) {
        return;
    }
    
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                g_bt_audio_instance->HandleA2dpConnected(param);
            } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                g_bt_audio_instance->HandleA2dpDisconnected(param);
            }
            break;
            
        case ESP_A2D_AUDIO_STATE_EVT:
            g_bt_audio_instance->HandleA2dpAudioState(param);
            break;
            
        case ESP_A2D_AUDIO_CFG_EVT:
            g_bt_audio_instance->HandleA2dpAudioConfig(param);
            break;
            
        default:
            ESP_LOGI(TAG, "Unhandled A2DP event: %d", event);
            break;
    }
}

void BluetoothAudioCodec::BtAppAvrcCallback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    ESP_LOGI(TAG, "AVRC event: %d", event);
}

void BluetoothAudioCodec::BtAppGapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            ESP_LOGI(TAG, "Discovered device: %02x:%02x:%02x:%02x:%02x:%02x",
                     param->disc_res.bda[0], param->disc_res.bda[1], param->disc_res.bda[2],
                     param->disc_res.bda[3], param->disc_res.bda[4], param->disc_res.bda[5]);
            break;
            
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            ESP_LOGI(TAG, "Discovery state changed: %d", param->disc_st_chg.state);
            break;
            
        default:
            break;
    }
}

void BluetoothAudioCodec::BtAppA2dDataCallback(const uint8_t *data, uint32_t len) {
    if (g_bt_audio_instance == nullptr || !g_bt_audio_instance->connected_ || g_bt_audio_instance->muted_) {
        return;
    }
    
    // 这里处理接收到的音频数据
    // 通常需要将数据发送到I2S接口或其他音频输出
    // 在实际应用中，可能需要进行格式转换、重采样等处理
    
    // 示例：直接将数据发送到I2S接口
    // size_t bytes_written = 0;
    // i2s_write(I2S_NUM_0, data, len, &bytes_written, portMAX_DELAY);
}

void BluetoothAudioCodec::HandleA2dpConnected(esp_a2d_cb_param_t *param) {
    ESP_LOGI(TAG, "A2DP connected to device: %02x:%02x:%02x:%02x:%02x:%02x",
             param->conn_stat.remote_bda[0], param->conn_stat.remote_bda[1],
             param->conn_stat.remote_bda[2], param->conn_stat.remote_bda[3],
             param->conn_stat.remote_bda[4], param->conn_stat.remote_bda[5]);
    
    connected_ = true;
    
    // 设置初始音量
    SetVolume(volume_);
}

void BluetoothAudioCodec::HandleA2dpDisconnected(esp_a2d_cb_param_t *param) {
    ESP_LOGI(TAG, "A2DP disconnected from device: %02x:%02x:%02x:%02x:%02x:%02x",
             param->conn_stat.remote_bda[0], param->conn_stat.remote_bda[1],
             param->conn_stat.remote_bda[2], param->conn_stat.remote_bda[3],
             param->conn_stat.remote_bda[4], param->conn_stat.remote_bda[5]);
    
    connected_ = false;
}

void BluetoothAudioCodec::HandleA2dpAudioState(esp_a2d_cb_param_t *param) {
    ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
}

void BluetoothAudioCodec::HandleA2dpAudioConfig(esp_a2d_cb_param_t *param) {
    ESP_LOGI(TAG, "A2DP audio config: sample rate=%d, channels=%d",
             param->audio_cfg.mcc.cie.sbc[0] & 0xF0,
             param->audio_cfg.mcc.cie.sbc[0] & 0x0F);
    
    // 根据蓝牙音频配置设置I2S接口
    // i2s_set_clk(I2S_NUM_0, sample_rate, I2S_BITS_PER_SAMPLE_16BIT, channels);
}
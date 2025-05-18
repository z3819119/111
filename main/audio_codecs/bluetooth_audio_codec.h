#pragma once

#include "audio_codec.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

class BluetoothAudioCodec : public AudioCodec {
public:
    BluetoothAudioCodec();
    virtual ~BluetoothAudioCodec();

    // 实现AudioCodec接口
    bool Initialize() override;
    bool Start() override;
    bool Stop() override;
    bool Pause() override;
    bool Resume() override;
    bool SetVolume(int volume) override;
    bool SetMute(bool mute) override;
    bool IsReady() const override;
    
    // 蓝牙特有方法
    bool StartDiscovery();
    bool StopDiscovery();
    bool Connect(const std::string& device_address);
    bool Disconnect();
    
private:
    // A2DP回调
    static void BtAppA2dCallback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
    static void BtAppAvrcCallback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
    static void BtAppGapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    
    // 数据回调
    static void BtAppA2dDataCallback(const uint8_t *data, uint32_t len);
    
    // 内部处理方法
    void HandleA2dpConnected(esp_a2d_cb_param_t *param);
    void HandleA2dpDisconnected(esp_a2d_cb_param_t *param);
    void HandleA2dpAudioState(esp_a2d_cb_param_t *param);
    void HandleA2dpAudioConfig(esp_a2d_cb_param_t *param);
    
    bool initialized_;
    bool connected_;
    int volume_;
    bool muted_;
};
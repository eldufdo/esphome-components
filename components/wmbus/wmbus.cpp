#include "wmbus.h"
#include "version.h"

#include "meters.h"

namespace esphome {
namespace wmbus {

  static const char *TAG = "wmbus";

  void WMBusComponent::setup() {
    this->high_freq_.start();
    if (this->led_pin_ != nullptr) {
      this->led_pin_->setup();
      this->led_pin_->digital_write(false);
      this->led_on_ = false;
    }
    if (!rf_mbus_.init(this->spi_conf_.mosi->get_pin(), this->spi_conf_.miso->get_pin(),
                      this->spi_conf_.clk->get_pin(), this->spi_conf_.cs->get_pin(),
                      this->spi_conf_.gdo0->get_pin(), this->spi_conf_.gdo2->get_pin(),
                      this->frequency_, this->sync_mode_)) {
      this->mark_failed();
      ESP_LOGE(TAG, "CC1101 initialization failed.");
      return;
    }

    // this->add_driver(new Amiplus());
    // this->add_driver(new Apator08());
    this->add_driver(new Apator162());
    // this->add_driver(new ApatorEITN());
    // this->add_driver(new Bmeters());
    // this->add_driver(new C5isf());
    // this->add_driver(new Compact5());
    // this->add_driver(new Dme07());
    // this->add_driver(new Elf());
    this->add_driver(new Evo868());
    // this->add_driver(new FhkvdataIII());
    // this->add_driver(new Flowiq2200());
    // this->add_driver(new Hydrocalm3());
    // this->add_driver(new Hydrus());
    // this->add_driver(new Iperl());
    // this->add_driver(new Itron());
    this->add_driver(new Izar());
    // this->add_driver(new Kamheat());
    // this->add_driver(new Mkradio3());
    // this->add_driver(new Mkradio4());
    // this->add_driver(new Mkradio4a());
    // this->add_driver(new Multical21());
    // this->add_driver(new Qheat());
    // this->add_driver(new Qwater());
    // this->add_driver(new Rfmtx1());
    // this->add_driver(new Sharky774("51728910E66D83F851728910E66D83F8"));
    // this->add_driver(new TopasESKR());
    // this->add_driver(new Ultrimis());
    // this->add_driver(new Unismart());
    // this->add_driver(new Vario451());
  }

  void WMBusComponent::loop() {
    this->led_handler();
    bool frameOk{true};
    // if (rf_mbus_.task()) {
    if (true) {
      ESP_LOGVV(TAG, "Have data from CC1101 ...");
      WMbusFrame mbus_data = rf_mbus_.get_frame();
      char frameMode[3]{0};
      frameMode[0] = 'T';
      frameMode[1] = '1';
      char frameFormat[2]{0};
      frameFormat[0] = 'A';
      std::vector<unsigned char> frame = {0x4E, 0x44, 0x24, 0x34, 0x68, 0x85, 0x00, 0x21, 0x50, 0x07, 0x7A, 0x82,
                                          0x33, 0x00, 0x20, 0x2F, 0x2F, 0x04, 0x13, 0x92, 0x9E, 0x01, 0x00, 0x04,
                                          0x6D, 0x1A, 0x37, 0x06, 0x32, 0x04, 0xFD, 0x17, 0x01, 0x41, 0x00, 0x00,
                                          0x0E, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x13, 0xB8, 0x49,
                                          0x01, 0x00, 0x42, 0x6C, 0xFF, 0x2C, 0x84, 0x01, 0x13, 0x53, 0x91, 0x01,
                                          0x00, 0x82, 0x01, 0x6C, 0x1F, 0x31, 0xD3, 0x01, 0x3B, 0xCF, 0x07, 0x00,
                                          0xC4, 0x01, 0x6D, 0x0D, 0x2D, 0xE5, 0x28};
      std::string telegram = format_hex_pretty(frame);
      telegram.erase(std::remove(telegram.begin(), telegram.end(), '.'), telegram.end());

      // ToDo: add manufactures check
      uint32_t meter_id = ((uint32_t)frame[7] << 24) | ((uint32_t)frame[6] << 16) |
                          ((uint32_t)frame[5] << 8)  | ((uint32_t)frame[4]);

      if (this->wmbus_listeners_.count(meter_id) > 0) {
        // for debug
        WMBusListener *text_debug{nullptr};
        if (this->wmbus_listeners_.count(0xAFFFFFF5) > 0) {
          text_debug = this->wmbus_listeners_[0xAFFFFFF5];
        }
        //
        auto *sensor = this->wmbus_listeners_[meter_id];
        if ( ((mbus_data.mode == 'T') && 
              ((sensor->framemode == MODE_T1) || (sensor->framemode == MODE_T1C1))) ||
            ((mbus_data.mode == 'C') &&
              ((sensor->framemode == MODE_C1) || (sensor->framemode == MODE_T1C1)))
            ) {
          auto selected_driver = this->drivers_[sensor->type];
          ESP_LOGD(TAG, "Using driver '%s' for ID [0x%08X] RSSI: %d dBm LQI: %d Frame: %s %s T: %s",
                  selected_driver->get_name().c_str(),
                  meter_id,
                  mbus_data.rssi,
                  mbus_data.lqi,
                  frameMode,
                  frameFormat,
                  telegram.c_str());
          //
          char hexString[8+1]{0};
          sprintf(hexString,"%08X", meter_id);
          ESP_LOGI(TAG, "key: '%s'", sensor->myKey.c_str());
          string id = std::string(hexString);
          //
          MeterInfo mi;
          mi.parse(selected_driver->get_name(), selected_driver->get_name(), id, sensor->myKey);
          auto meter = createMeter(&mi);
          AboutTelegram about;
          bool id_match = false;
          meter->handleTelegram(about, frame, false, &id, &id_match, NULL);
          double val = meter->getNumericValue("total", Unit::M3);
          ESP_LOGI(TAG, "Mamy z wmbusmeters: %.6f", val);
          val = meter->getNumericValue("total_m3", Unit::M3);
          ESP_LOGI(TAG, "Mamy z wmbusmeters: %.6f", val);
          //
          if (sensor->key.size()) {
            ESP_LOGVV(TAG, "Key defined, trying to decrypt telegram ...");
            if (this->decrypt_telegram(frame, sensor->key)) {
              std::string decrypted_telegram = format_hex_pretty(frame);
              decrypted_telegram.erase(std::remove(decrypted_telegram.begin(), decrypted_telegram.end(), '.'),
                                      decrypted_telegram.end());
              ESP_LOGD(TAG, "Decrypted T : %s", decrypted_telegram.c_str());
            }
            else {
              frameOk = false;
            }
          }
          if (frameOk) {
            auto mapValues = selected_driver->get_values(frame);
            if (mapValues.has_value()) {
              if (this->wmbus_listeners_[meter_id]->sensors_.count("lqi") > 0) {
                this->wmbus_listeners_[meter_id]->sensors_["lqi"]->publish_state(mbus_data.lqi);
              }
              if (this->wmbus_listeners_[meter_id]->sensors_.count("rssi") > 0) {
                this->wmbus_listeners_[meter_id]->sensors_["rssi"]->publish_state(mbus_data.rssi);
              }
              for (const auto &ele : mapValues.value()) {
                if (this->wmbus_listeners_[meter_id]->sensors_.count(ele.first) > 0) {
                  ESP_LOGV(TAG, "Publishing '%s' = %.4f", ele.first.c_str(), ele.second);
                  this->wmbus_listeners_[meter_id]->sensors_[ele.first]->publish_state(ele.second);
                }
                // for debug
                if (text_debug != nullptr) {
                  if (((this->wmbus_listeners_[meter_id]->type == "apator162") &&
                      (this->wmbus_listeners_[meter_id]->sensors_.count("total_water_m3") > 0) &&
                      (ele.second > 500000)) ||
                      ((this->wmbus_listeners_[meter_id]->type == "apatoreitn") &&
                      (this->wmbus_listeners_[meter_id]->sensors_.count("current_hca") > 0) &&
                      (ele.second > 400))) {
                    text_debug->text_sensor_->publish_state("apator strange value");
                    std::string telegramik;
                    int split = 100;
                    int start = 0;
                    int part = 1;
                    while (start < telegram.size()) {
                      telegramik = std::to_string(part++) + "  | ";
                      telegramik += telegram.substr(start, split);
                      text_debug->text_sensor_->publish_state(telegramik);
                      start += split;
                    }
                    std::string decoded_telegramik = format_hex_pretty(frame);
                    split = 75;
                    start = 0;
                    part = 1;
                    while (start < decoded_telegramik.size()) {
                      telegramik = std::to_string(part++) + "' | ";
                      telegramik += decoded_telegramik.substr(start, split);
                      text_debug->text_sensor_->publish_state(telegramik);
                      start += split;
                      split = 99;
                    }
                  }
                }
                //
              }
              this->led_blink();
            }
            else {
              ESP_LOGD(TAG, "Can't get value(s) from telegram for ID [0x%08X]", meter_id);
            }
          }
        }
      }
      else {
        if (this->wmbus_listeners_.count(0) > 0) {
          if (this->wmbus_listeners_[0]->sensors_.count("lqi") > 0) {
            this->wmbus_listeners_[0]->sensors_["lqi"]->publish_state(mbus_data.lqi);
          }
          if (this->wmbus_listeners_[0]->sensors_.count("rssi") > 0) {
            this->wmbus_listeners_[0]->sensors_["rssi"]->publish_state(mbus_data.rssi);
          }
        }
        if (this->log_unknown_) {
          ESP_LOGD(TAG, "Meter ID [0x%08X] RSSI: %d dBm LQI: %d Frame: %s %s not found in configuration T: %s",
                  meter_id,
                  mbus_data.rssi,
                  mbus_data.lqi,
                  frameMode,
                  frameFormat,
                  telegram.c_str());
        }
      }
      if (!(this->clients_.empty())) {
        ESP_LOGVV(TAG, "Will send telegram to clients ...");
        this->led_blink();
      }
    }
    delay(10);
  }

  bool WMBusComponent::decrypt_telegram(std::vector<unsigned char> &telegram, std::vector<unsigned char> &key) {
    bool ret_val = true;
    int ci_field = telegram[10];
    switch(ci_field) {
      case 0x8D:
        {
          if (decrypt_ELL_AES_CTR(telegram, key)) {
            static const uint8_t offset{17};
            uint8_t payload_len = telegram.size() - 2 - offset;  // telegramFrameSize - CRC - offset
            ESP_LOGV(TAG, "Validating CRC for ELL payload");
            if (!crcValid((safeButUnsafeVectorPtr(telegram) + offset), 0, payload_len)) {
              ret_val = false;
            }
          }
          else {
            ESP_LOGVV(TAG, "Decrypting ELL AES CTR failed!");
            ret_val = false;
          }
        }
        break;

      default:
        {
          if (!decrypt_TPL_AES_CBC_IV(telegram, key)) {
            ESP_LOGVV(TAG, "Decrypting TPL AES CBC IV failed!");
            ret_val = false;
          }
        }
        break;
    }
    return ret_val;
  }

  void WMBusComponent::register_wmbus_listener(WMBusListener *listener) {
    this->wmbus_listeners_[listener->id] = listener;
  }

  void WMBusComponent::add_driver(Driver *driver) {
    this->drivers_[driver->get_name()] = driver;
  }

  void WMBusComponent::led_blink() {
    if (this->led_pin_ != nullptr) {
      if (!this->led_on_) {
        this->led_on_millis_ = millis();
        this->led_pin_->digital_write(true);
        this->led_on_ = true;
      }
    }
  }

  void WMBusComponent::led_handler() {
    if (this->led_pin_ != nullptr) {
      if (this->led_on_) {
        if ((millis() - this->led_on_millis_) >= this->led_blink_time_) {
          this->led_pin_->digital_write(false);
          this->led_on_ = false;
        }
      }
    }
  }

  const LogString *framemode_to_string(FrameMode framemode) {
    switch (framemode) {
      case MODE_T1:
        return LOG_STR("T1");
      case MODE_C1:
        return LOG_STR("C1");
      case MODE_T1C1:
        return LOG_STR("T1C1");
      default:
        return LOG_STR("");
    }
  }

  const LogString *WMBusComponent::format_to_string(Format format) {
    switch (format) {
      case FORMAT_HEX:
        return LOG_STR("hex");
      case FORMAT_RTLWMBUS:
        return LOG_STR("rtl-wmbus");
      default:
        return LOG_STR("unknown");
    }
  }

  const LogString *WMBusComponent::transport_to_string(Transport transport) {
    switch (transport) {
      case TRANSPORT_TCP:
        return LOG_STR("TCP");
      case TRANSPORT_UDP:
        return LOG_STR("UDP");
      default:
        return LOG_STR("unknown");
    }
  }

  void WMBusComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "wM-Bus v%s:", MY_VERSION);
    if (this->led_pin_ != nullptr) {
      ESP_LOGCONFIG(TAG, "  LED:");
      LOG_PIN("    Pin: ", this->led_pin_);
      ESP_LOGCONFIG(TAG, "    Duration: %d ms", this->led_blink_time_);
    }
    ESP_LOGCONFIG(TAG, "  CC1101 frequency: %3.3f MHz", this->frequency_);
    ESP_LOGCONFIG(TAG, "  CC1101 SPI bus:");
    LOG_PIN("    MOSI Pin: ", this->spi_conf_.mosi);
    LOG_PIN("    MISO Pin: ", this->spi_conf_.miso);
    LOG_PIN("    CLK Pin:  ", this->spi_conf_.clk);
    LOG_PIN("    CS Pin:   ", this->spi_conf_.cs);
    LOG_PIN("    GDO0 Pin: ", this->spi_conf_.gdo0);
    LOG_PIN("    GDO2 Pin: ", this->spi_conf_.gdo2);
    if (this->drivers_.size() > 0) {
      std::string drivers = "  ";
      for (const auto& element : this->drivers_) {
        drivers += element.first + ", ";
      }
      drivers.erase(drivers.size() - 2);
      ESP_LOGCONFIG(TAG, "  Available drivers:%s", drivers.c_str());
      for (const auto &ele : this->wmbus_listeners_) {
        ele.second->dump_config();
      }
    }
    else {
      ESP_LOGE(TAG, "  Check connection to CC1101!");
    }
  }

  ///////////////////////////////////////

  void WMBusListener::dump_config() {
    std::string key = format_hex_pretty(this->key);
    key.erase(std::remove(key.begin(), key.end(), '.'), key.end());
    if (key.size()) {
      key.erase(key.size() - 5);
    }
    ESP_LOGCONFIG(TAG, "  Meter:");
    ESP_LOGCONFIG(TAG, "    ID: %zu [0x%08X]", this->id, this->id);
    ESP_LOGCONFIG(TAG, "    Type: %s", this->type.c_str());
    ESP_LOGCONFIG(TAG, "    Mode: %s", LOG_STR_ARG(framemode_to_string(this->framemode)));
    ESP_LOGCONFIG(TAG, "    Key: '%s'", key.c_str());
    for (const auto &ele : this->sensors_) {
      LOG_SENSOR("    ", "Sensor", ele.second);
    }
  }

  WMBusListener::WMBusListener(const uint32_t id, const std::string type, const std::string key, const FrameMode framemode) {
    this->id = id;
    this->type = type;
    this->framemode = framemode;
    this->myKey = key;
    hex_to_bin(key, &(this->key));
  }

  WMBusListener::WMBusListener(const uint32_t id, const std::string type, const std::string key) {
    this->id = id;
    this->type = type;
    this->myKey = key;
    hex_to_bin(key, &(this->key));
  }

  int WMBusListener::char_to_int(char input)
  {
    if(input >= '0' && input <= '9') {
      return input - '0';
    }
    if(input >= 'A' && input <= 'F') {
      return input - 'A' + 10;
    }
    if(input >= 'a' && input <= 'f') {
      return input - 'a' + 10;
    }
    return -1;
  }

  bool WMBusListener::hex_to_bin(const char* src, std::vector<unsigned char> *target)
  {
    if (!src) return false;
    while(*src && src[1]) {
      if (*src == ' ' || *src == '#' || *src == '|' || *src == '_') {
        // Ignore space and hashes and pipes and underlines.
        src++;
      }
      else {
        int hi = char_to_int(*src);
        int lo = char_to_int(src[1]);
        if (hi<0 || lo<0) return false;
        target->push_back(hi*16 + lo);
        src += 2;
      }
    }
    return true;
  }

}  // namespace wmbus
}  // namespace esphome

#ifndef ROBO_CV_COMMPORT_H
#define ROBO_CV_COMMPORT_H
#include <Checksum.h>
#include <serial/serial.h>
#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

constexpr size_t packet_size = 15;

struct TxPacket {
  unsigned char cache[packet_size];

  unsigned char &operator[](int p) { return cache[p]; }

  unsigned char operator[](int p) const { return cache[p]; }

  unsigned char *operator&() { return cache; }

  /*Constructor*/
  TxPacket() { memset(cache, 0, sizeof(0)); }
};

class CommPort {
private:
  std::atomic<bool> read_stop_flag_{};
  std::atomic<bool> write_stop_flag_{};
  std::atomic<bool> write_clear_flag_{};
  std::atomic<bool> exception_handled_flag_{};
  serial::Serial port_;

  std::shared_ptr<spdlog::logger> logger_;
  std::vector<serial::PortInfo> serial_port_info_;
  std::string device_desc_;

public:
  uint8_t tx_buffer_[64]{};  // TODO: bigger buffer?
  uint8_t rx_buffer_[256]{}; // TODO: BIGGer buffer?

  typedef struct __attribute__((packed)) {
    uint8_t header;
    float q[4];

    float yaw;
    float pitch;

    uint8_t lever_mode;
    // the lever_mode show the mode of right lever on the remote controller
    // SW_UP -> 1 SW_MID = 2 SW_DOWN = 3

    float bullet_speed;
    // NOTE: Now the bullet_speed is added through the shoot_data and
    // _bullet_speed_pub_

    uint8_t checksum;
  } ProjectileRx_fast;

  typedef struct ProjectileRx_slow {
    uint8_t header;

    // TODO: WHAT IS THIS?
    uint32_t timestamp;

    // NOTE: the mode can be set as any value you want
    uint8_t vision_mode; // default -> CLASSIC 0, (WIND -> 1)

    // the gurgement part (coming from c borad)
    uint8_t shoot_remote;
    // TODO: WTF is the one things above

    // the current side color (which team we are)
    // 0 -> unknown
    // 1 -> red
    // 2 -> blue
    uint8_t current_side_color;
    // the enemt_hp
    uint16_t enemy_1_robot_HP;
    uint16_t enemy_2_robot_HP;
    uint16_t enemy_3_robot_HP;
    uint16_t enemy_4_robot_HP;
    uint16_t enemy_5_robot_HP;
    uint16_t enemy_7_robot_HP;
    uint16_t enemy_outpost_HP;
    uint16_t enemy_base_HP;

    // field events
    uint32_t field_events;

    // game status
    uint8_t game_type;
    uint8_t game_progress;
    uint16_t state_remain_time;
    uint64_t sync_time_stamp;

    // game result
    uint8_t winner;

    // robot buffs
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;

    // robot position
    float x;
    float y;
    float angle;

    // robot status
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;

    // ext_power_heat_data_t
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;

    // client command
    float command_target_position[3];
    // float target_position_x;
    // float target_position_y;
    // float target_position_z;
    uint16_t keyboard_key_pressed;
    uint8_t command_target_robot_id;

    // client receive
    float receive_target_position[2];
    // float target_position_x;
    // float target_position_y;
    uint8_t receive_target_robot_id;

  } ProjectileRx_slow;

  typedef struct ProjectileTx {
    uint8_t header;
    // original rx vision
    float yaw_angle;
    float pitch_angle;

    // autoaim
    float pitch_actual;
    uint8_t raw1[4];
    float yaw_actual;
    uint8_t raw2[4];
    // navigation
    float linear_x;
    float linear_y;
    float angular_z;

    uint8_t target_found; // the autofire is to tell the robot to fire
    uint8_t placeholder2;
    uint8_t placeholder3;

    uint8_t checksum;
  } ProjectileTx;

  ProjectileTx tx_struct_{};

  ProjectileRx_fast rx_struct_fast_{};
  ProjectileRx_slow rx_struct_slow_{};

  enum SERIAL_MODE { TX_SYNC, TX_RX_ASYNC };

  CommPort();

  ~CommPort();

  void RunAsync(SERIAL_MODE mode);

  void Start();

  void Stop();

  void Write(const uint8_t *tx_packet, size_t size, bool safe_write);

  void Read();

  void RxHandler_fast();

  void RxHandler_slow();

  void tx_struct_dump(uint8_t *tx_buffer);

  void SerialFailsafeCallback(bool reopen);
};

#endif // ROBO_CV_COMMPORT_H

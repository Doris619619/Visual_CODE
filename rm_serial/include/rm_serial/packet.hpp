#ifndef SERIAL_DRIVER__PACKET_HPP_
#define SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
	// down packet
	struct DownReceivePacket
	{
		uint8_t header = 0x5A;
		uint8_t detect_color : 1; // 0-red 1-blue
		bool reset_tracker : 1;
		uint8_t reserved : 6;

		float yaw;    // rad
		float pitch;  // rad
		float roll;   // rad
		float yaw_odom;
		float pitch_odom;
		float yaw_vel;    // rad/s
		float pitch_vel;  // rad/s
		float roll_vel;   // rad/s	

		float aim_x; // 瞄准的点的位置
		float aim_y;
		float aim_z;
		uint8_t robot_id;

		uint16_t checksum = 0;
	} __attribute__((packed));

	struct SendPacket
	{
		uint8_t header = 0xA5;
		bool tracking : 1;		// 0-not detect, 1-detect target
		// uint8_t id : 3;			// 0-outpost 6-guard 7-base
		// uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
		// uint8_t reserved : 1;
		// float x; // target position
		// float y;
		// float z;
		// float yaw; // armor targeting angle
		// float vx;  // target velocity
		// float vy;
		// float vz;
		// float v_yaw;
		// float r1; // 椭圆的长轴和短轴
		// float r2;s
		// float dz;
		float pitch;
		float yaw;
		uint8_t fire;
		uint8_t fric_on;

		uint16_t checksum = 0; // CRC校验的冗余码

	} __attribute__((packed));

	// inline DownReceivePacket fromDownVector(const std::vector<uint8_t> &data)
	// {
	// 	DownReceivePacket packet;
	// 	std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
	// 	return packet;
	// }

	// inline std::vector<uint8_t> toVector(const SendPacket &data)
	// {
	// 	std::vector<uint8_t> packet(sizeof(SendPacket));
	// 	std::copy(
	// 		reinterpret_cast<const uint8_t *>(&data),
	// 		reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
	// 	return packet;
	// }

	// ext packet
	struct EXTReceivePacket
	{
		uint8_t header = 0x68;

		uint8_t robot_id;

		uint16_t checksum = 0;
	} __attribute__((packed));

	/********************************************************/
	/* template                                             */
	/********************************************************/

	template <typename T>
	inline T fromVector(const std::vector<uint8_t> &data)
	{
		T packet;
		std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
		return packet;
	}

	template <typename T>
	inline std::vector<uint8_t> toVector(const T &data)
	{
		std::vector<uint8_t> packet(sizeof(T));
		std::copy(
			reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
			packet.begin());
		return packet;
	}

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__PACKET_HPP_

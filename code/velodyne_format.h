#if !defined __BAH_VELODYNE_FORMAT_H__
#define __BAH_VELODYNE_FORMAT_H__

struct SingleLaserMeasure
{
    uint16_t range;
    uint8_t intensity;
}
__attribute__((packed));

struct LaserBankMeasure
{
    uint16_t start_identifier;
    uint16_t angle_hundredths;
    SingleLaserMeasure laser_measures[32];
}
__attribute__((packed));

struct LaserPacket
{
    LaserBankMeasure laser_bank_measures[12];
    uint32_t gps_timestamp;
    uint8_t unused1;
    uint8_t unused2;
}
__attribute__((packed));

#endif

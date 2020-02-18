#ifndef __USB_MIXER_ESS014_H__
#define __USB_MIXER_ESS014_H__

/* ESS014 USB Packet */
struct snd_ess014_packet_header;
struct snd_ess014_packet;

#define SND_ESS014_MAX_PACKET_SIZE	64
#define SND_ESS014_PACKET_HDR_SIZE	sizeof(struct snd_ess014_packet_header)
#define SND_ESS014_PACKET_DATA_SIZE	(SND_ESS014_MAX_PACKET_SIZE - \
					 SND_ESS014_PACKET_HDR_SIZE)

#define SND_ESS014_OPCODE_READ		0x1D
#define SND_ESS014_OPCODE_WRITE		0x1F
#define SND_ESS014_ADDR_MSB		0x80

struct snd_ess014_packet_header {
	u8 opcode;
	struct {
		u8 msb;
		u8 lsb;
	} addr;
	u8 length;
};

struct snd_ess014_packet {
	struct snd_ess014_packet_header hdr;
	u8 data[SND_ESS014_PACKET_DATA_SIZE];
};

/* ESS014 Registers */
#define SND_ESS014_REG_MAX				0xDC

#define SND_ESS014_FILTER_SETTINGS_REG			0x34
#define SND_ESS014_DAC_FILTER_SHAPE_MASK		GENMASK(2, 0)
#define SND_ESS014_DAC_FILTER_FAST_ROLE_OFF		0x00
#define SND_ESS014_DAC_FILTER_SLOW_ROLE_OFF_MIN_PHASE	0x03
#define SND_ESS014_DAC_FILTER_MIN_PHASE_HYBRID		0x06
#define SND_ESS014_DAC_FILTER_CUSTOM			0x0E

#define SND_ESS014_ANALOG_GAIN_CONTROL_1_REG		0x51
#define SND_ESS014_DAC_ANALOG_USE_SECONDARY_BIT		BIT(7)
#define SND_ESS014_DAC_ANALOG_1DB_ENB_BIT		BIT(5)
#define SND_ESS014_DAC_ANALOG_VOLUME_MASK		GENMASK(4, 0)

#define SND_ESS014_ANALOG_GAIN_CONTROL_2_REG		0x52

#define SND_ESS014_CHIP_ID_REG				0xDC
#define SND_ESS014_CHIP_ID_MASK				GENMASK(6, 0)

int snd_ess014_controls_create(struct usb_mixer_interface *mixer);

#endif /* __USB_MIXER_ESS014_H__ */

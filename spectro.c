/**
 * @file
 * @author Philip Pemberton <philpem@philpem.me.uk>
 *
 * Spectro -- a simple tool to poll the Colorvision Spyder3Print (Datacolor
 * 1005) spectrocolorimiter.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <libusb-1.0/libusb.h>

/////////////////////////////////////////////////////////////////////////////////////

/// Vendor ID used by the CM3
#define VID_COLORVISION 0x085C
/// Product ID used by the CM3
#define PID_COLORMOUSE3 0x0003

/// CM3 commands
typedef enum {
	CM3_CMD_GET_FIRMWARE_VERSION	= 0x02,		///< Get firmware version (returns 4 bytes)
	CM3_CMD_SOFT_RESET				= 0x04,		///< Soft reset (returns nothing)
	CM3_CMD_MEASURE					= 0x0A,		///< Measure
	CM3_CMD_CALIBRATE_WHITE			= 0x12,		///< Calibrate white level
	CM3_CMD_READ_MEMORY				= 0x16,		///< Read from CM3 memory
	CM3_CMD_WRITE_MEMORY			= 0x17		///< Write to CM3 memory
} CM3_COMMAND;

/// Calibration modes
typedef enum {
	CM3_CAL_WHITE					= 0,		///< Calibrate from white tile
	CM3_CAL_BLACK					= 1			///< Calibrate from black tile (not used)
} CM3_CAL_TYPE;

/////////////////////////////////////////////////////////////////////////////////////

/**
 * Send a command to the CM3.
 *
 * @param	buf		Target buffer
 * @param	cmd		Command code
 * @param	len		Payload length (or zero if no payload)
 * @return	Status code, or -1 on error
 */
int cm3_cmd(libusb_device_handle *devh, CM3_COMMAND cmd, unsigned char *payload, int *len, unsigned char *response)
{
	unsigned char buf[256];
	static unsigned char cm3_command_tag = 0;
	unsigned char tag_this, status;
	int err, pos, count;

	assert(len != NULL);
	assert(*len >= 0);
	assert(*len < ((int)sizeof(buf)-3));

	/***
	 * Packet format:
	 * <TAG> <LEN> <CMD> <PAYLOAD>
	 */
	buf[0] = tag_this = cm3_command_tag;
	buf[1] = *len + 3;  // Counts the 3-byte header too
	buf[2] = cmd;

	cm3_command_tag = (cm3_command_tag + 1) & 0x7F;

	// Copy in the payload
	if (*len > 0) {
		memcpy(&buf[3], payload, *len);
	}

	// Send command
	pos = 0;
	while (pos < (*len + 3)) {
		// 1-sec timeout
		err = libusb_bulk_transfer(devh, 2 | LIBUSB_ENDPOINT_OUT, &buf[pos], (*len + 3) - pos, &count, 1000);
		if ((err != 0) && (err != LIBUSB_ERROR_TIMEOUT)) {
			fprintf(stderr, "Error %d (%s) during EP OUT command transfer\n", err, libusb_error_name(err));
			return -1;
		}
		pos += count;
	}

	// Receive response
	err = libusb_bulk_transfer(devh, 2 | LIBUSB_ENDPOINT_IN, buf, sizeof(buf), &count, 1000);
	if (err != 0) {
		fprintf(stderr, "Error %d (%s) during EP IN transfer\n", err, libusb_error_name(err));
		return -1;
	}

	// Make sure tag matches the one we sent
	if (buf[0] != tag_this) {
		fprintf(stderr, "WARNING: Mismatched tag in response\n");
	}

	// Save status code and length
	status = buf[2];
	*len = buf[1] - 3;

	// Copy the rest of the payload (if any)
	if ((*len > 0) && (response != NULL)) {
		memcpy(response, &buf[3], *len);
	}

	return status;
}

/**
 * Soft reset
 */
int cm3_soft_reset(libusb_device_handle *devh)
{
	int len = 0;

	return cm3_cmd(devh, CM3_CMD_SOFT_RESET, NULL, &len, NULL);
}

/**
 * Read firmware version
 */
int cm3_get_fw_ver(libusb_device_handle *devh, unsigned int *ver)
{
	int len = 0;
	int err;
	unsigned char buf[32];

	err = cm3_cmd(devh, CM3_CMD_GET_FIRMWARE_VERSION, NULL, &len, buf);
	if (err == 0) {
		*ver = 0;
		for (int i=0; i<4; i++) {
			*ver = (*ver << 8) + buf[i];
		}
	}

	return err;
}

/**
 * Read CM3 memory.
 */
int cm3_read_memory(libusb_device_handle *devh, unsigned int addr, int len, unsigned char *rxbuf)
{
	unsigned char txbuf[6];
	int rlen;

	assert(len > 0);

	// Prepare read command packet
	txbuf[0] = (addr >> 16) & 0xff;
	txbuf[1] = (addr >>  8) & 0xff;
	txbuf[2] = (addr)       & 0xff;
	txbuf[3] = (len  >>  8) & 0xff;
	txbuf[4] = (len)        & 0xff;

	rlen = 5;
	return cm3_cmd(devh, CM3_CMD_READ_MEMORY, txbuf, &rlen, rxbuf);
}

int cm3_get_profile_size(libusb_device_handle *devh, unsigned int *psize)
{
	unsigned char buf[2];
	int err;

	if ((err = cm3_read_memory(devh, 0x8000B, 2, buf)) != 0) {
		return err;
	}

	*psize = (buf[0] << 8) | buf[1];

	return 0;
}

int cm3_get_num_bands(libusb_device_handle *devh, unsigned int *nbands)
{
	unsigned char buf[2];
	int err;

	if ((err = cm3_read_memory(devh, 0x80018, 1, buf)) != 0) {
		return err;
	}

	*nbands = buf[0];

	return 0;
}

int cm3_get_output_mode(libusb_device_handle *devh, unsigned char *output_mode)
{
	unsigned int psize, nbands;
	int err;

	if ((err = cm3_get_profile_size(devh, &psize)) != 0)
		return err;

	if ((err = cm3_get_num_bands(devh, &nbands)) != 0)
		return err;

	if (psize > 64) {
		err = cm3_read_memory(devh, 0x7FFFB + psize - (8*nbands), 1, output_mode);
		if (err) return err;
	} else {
		*output_mode = 0;
	}

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
	libusb_context *ctx = NULL;
	libusb_device_handle *devh;
	int err;

	// Initialise libusb
	if ((err = libusb_init(&ctx)) != 0) {
		fprintf(stderr, "Error %d initialising libusb\n", err);
		return EXIT_FAILURE;
	}

	// Connect to device
	devh = libusb_open_device_with_vid_pid(ctx, VID_COLORVISION, PID_COLORMOUSE3);
	if (err) {
		fprintf(stderr, "Unable to open USB device, error %d (%s)\n", err, "??err??");	// FIXME use libusb to find out what the error was
		libusb_exit(ctx);
		return EXIT_FAILURE;
	}

	if (devh == NULL) {
		fprintf(stderr, "No Datacolor 1005 / Spyder3Print / Color Mouse III spectrophotometers found.\n");
		libusb_exit(ctx);
		return EXIT_FAILURE;
	}

	// Claim interface
	if ((err = libusb_claim_interface(devh, 0)) != 0) {
		fprintf(stderr, "Error %d while claiming interface 0.\n", err);
		libusb_close(devh);
		libusb_exit(ctx);
		return EXIT_FAILURE;
	}

	// device now open
	printf("CM3 Reset: %d\n", cm3_soft_reset(devh));

	unsigned int fwv;
	err = cm3_get_fw_ver(devh, &fwv);
	printf("CM3 Get FW Version: %d ==> 0x%08X\n", err, fwv);

/***
 * Initialisation:
 *
 * Get output mode
 * Get measurement table
 * Measure
 */

	unsigned char outmode;
	err = cm3_get_output_mode(devh, &outmode);
	printf("CM3 Output Mode => %d\n", outmode);

	// output modes --
	// 		0x10 = LAB D50
	// 		0x11 = LAB D65
	// 		0x20 = XYZ D50
	// 		0x21 = XYZ D65

	libusb_release_interface(devh, 0);
	libusb_close(devh);
	libusb_exit(ctx);
	return 0;
}

// vim:ts=4 sw=4

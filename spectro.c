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

/////////////////////////////////////////////////////////////////////////////////////

/**
 * Send a command to the CM3.
 *
 * @param	buf		Target buffer
 * @param	cmd		Command code
 * @param	len		Payload length (or zero if no payload)
 * @return	Status code, or -1 on error
 */
int cm3_cmd(libusb_device_handle *devh, unsigned char cmd, unsigned char *payload, int *len, unsigned char *response)
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
	int len;
	len = 0;
	err = cm3_cmd(devh, CM3_CMD_SOFT_RESET, NULL, &len, NULL);
	printf("CM3 Reset: %d\n", err);

	unsigned char buf[32];
	len = 0;
	err = cm3_cmd(devh, CM3_CMD_GET_FIRMWARE_VERSION, NULL, &len, buf);
	printf("CM3 Get FW Version: %d", err, len);
	if (err == 0) {
		unsigned long foo = 0;
		for (int i=0; i<4; i++) foo = (foo << 8) + buf[i];
		printf(" = 0x%08X\n", foo);
	} else {
		printf("\n");
	}

	// meas types --
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

/**
 * @file
 * @author Philip Pemberton <philpem@philpem.me.uk>
 *
 * Spectro -- a simple tool to poll the Colorvision Spyder3Print (Datacolor
 * 1005) spectrocolorimiter.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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

typedef struct {
	int profile_size, num_bands, output_mode, meas_table_len;
	int profile_size_valid, num_bands_valid, output_mode_valid, meas_table_len_valid;
} CM3_STATE;

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

int cm3_get_output_mode(libusb_device_handle *devh, CM3_STATE *state, unsigned char *output_mode)
{
	unsigned int psize, nbands;
	int err;

	// Get the profile size if we don't already have it
	if (!state->profile_size_valid) {
		if ((err = cm3_get_profile_size(devh, &psize)) != 0) {
			return err;
		}
		state->profile_size = psize;
		state->profile_size_valid = true;
	}

	// Do the same for number of bands
	if (!state->num_bands_valid) {
		if ((err = cm3_get_num_bands(devh, &nbands)) != 0) {
			return err;
		}
		state->num_bands = nbands;
		state->num_bands_valid = true;
	}

	// Profile size > 64 means the output mode is stored in memory, else it isn't
	if (psize > 64) {
		err = cm3_read_memory(devh, 0x7FFFB + state->profile_size - (8 * state->num_bands), 1, output_mode);
		if (err) {
			return err;
		}
	} else {
		*output_mode = 0;
	}

	state->output_mode = *output_mode;
	state->output_mode_valid = true;

	return 0;
}

int cm3_get_measurement_table(libusb_device_handle *devh, CM3_STATE *state, unsigned char *table)
{
	unsigned int nbands;
	unsigned char mtl;
	int err;

	// Need number of bands to calculate memory offset
	if (!state->num_bands_valid) {
		if ((err = cm3_get_num_bands(devh, &nbands)) != 0) {
			return err;
		}
		state->num_bands = nbands;
		state->num_bands_valid = true;
	} else {
		nbands = state->num_bands;
	}

	// Read measurement table length
	if ((err = cm3_read_memory(devh, 0x8001A + (2 * state->num_bands), 1, &mtl)) != 0) {
		return err;
	}

	// Set valid flag, store measurement table length
	state->meas_table_len_valid = true;
	state->meas_table_len = mtl;

	// Read table if we need it
	if (table) {
		*table = mtl;
		if ((err = cm3_read_memory(devh, 0x8001B + (2 * state->num_bands), state->meas_table_len, &mtl)) != 0) {
			return err;
		}
	}

	return 0;
}

typedef struct {
	char mode;
	float x,y,z;
	float l,a,b;
} CM3_MEASUREMENT;

int cm3_measure(libusb_device_handle *devh, CM3_STATE *state, CM3_MEASUREMENT *meas)
{
	unsigned char output_mode;
	unsigned char rxbuf[32];
	int err, len;

	// Send "Measure" command
	len = 0;
	if ((err = cm3_cmd(devh, CM3_CMD_MEASURE, NULL, &len, rxbuf)) != 0) {
		return err;
	}

	// Get output mode and measurement table length
	if (!state->output_mode_valid) {
		if ((err = cm3_get_output_mode(devh, state, &output_mode)) != 0) {
			return err;
		}
	}

	if (!state->meas_table_len_valid) {
		if ((err = cm3_get_measurement_table(devh, state, NULL)) != 0) {
			return err;
		}
	}

	printf("Meastable Len %d\n", state->meas_table_len);
	printf("Read Len %d\n", len);

	if (len == 12) {
		// Length of 12 -- Lab colour
		long lab_buf[3];
		for (int i=0; i<3; i++) {
			lab_buf[i] =
				(rxbuf[(4*i)] << 24) +
				(rxbuf[(4*i)+1] << 16) +
				(rxbuf[(4*i)+2] << 8) +
				(rxbuf[(4*i)+3]);
		}

		// Load output buffer
		if (lab_buf[0] >= 0x7FE89) {
			meas->l = ((float)lab_buf[0] / 65535.0) - 16.0;
		} else {
			meas->l = ((float)lab_buf[0] / 65535.0);
		}
		meas->a = (float)lab_buf[1] / 65535.0;
		meas->b = (float)lab_buf[2] / 65535.0;
	} else if (len == 6) {
		// Length of 6 -- XYZ colour
		int xyz_buf[3];
		for (int i=0; i<3; i++) {
			xyz_buf[i] =
				(rxbuf[(2*i)] << 8) +
				(rxbuf[(2*i)+1]);
		}

		// Load output buffer
		if (output_mode == 0x20) {
			// D50 XYZ
			meas->x = xyz_buf[0] * 96.420998 / 65535.0;
			meas->y = xyz_buf[1] * 100.0 / 65535.0;
			meas->z = xyz_buf[2] * 82.524002 / 65535.0;
		} else if (output_mode == 0x21) {
			// D65 XYZ
			meas->x = xyz_buf[0] * 94.808998 / 65535.0;
			meas->y = xyz_buf[1] * 100.0 / 65535.0;
			meas->z = xyz_buf[2] * 107.307 / 65535.0;
		} else {
			// error
			return -1001;
		}
	} else {
		return -1000;
	}

	return 0;
}

// TODO
// TODO
// Get Unit ID
// Calibrate
// Poll Button (interrupt EP 1)
// TODO
// TODO

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
 * Get measurement table len
 *
 * Measure
 * Calibrate
 *
 * Measure [...]
 */

	CM3_STATE state;
	memset(&state, 0, sizeof(state));

	unsigned char outmode;
	err = cm3_get_output_mode(devh, &state, &outmode);
	printf("CM3 Output Mode => %d\n", outmode);

	CM3_MEASUREMENT meas;
	err = cm3_measure(devh, &state, &meas);
	printf("CM3 Measure => %d\n", err);
	printf("Output mode: ");
	switch (state.output_mode) {
		case 0x10: printf("LAB\n"); break;
		case 0x20: printf("XYZ D50\n"); break;
		case 0x21: printf("XYZ D65\n"); break;
	}

	switch (state.output_mode & 0xF0) {
		case 0x10:
			printf("\tL = %f\n", meas.l);
			printf("\ta = %f\n", meas.a);
			printf("\tb = %f\n", meas.b);
			break;
		case 0x20:
			printf("\tX = %f\n", meas.x);
			printf("\tY = %f\n", meas.y);
			printf("\tZ = %f\n", meas.z);
			break;
	}

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

/*
Copyright (c) 2015, Raspberry Pi Foundation
Copyright (c) 2015, Dave Stevenson
Copyright (c) 2020, Thomas Oldbury
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define VERSION_STRING "0.0.2"

#define _GNU_SOURCE
#include <ctype.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#define I2C_SLAVE_FORCE 0x0706
#include "interface/vcos/vcos.h"
#include "bcm_host.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiCLI.h"

#include <sys/ioctl.h>

#include "raw_header.h"

#define DEFAULT_I2C_DEVICE 0

#define I2C_DEVICE_NAME_LEN 13	// "/dev/i2c-XXX"+NULL

int packet_idx = 0;

static char i2c_device_name[I2C_DEVICE_NAME_LEN];

struct brcm_raw_header *brcm_header = NULL;

enum bayer_order {
	//Carefully ordered so that an hflip is ^1,
	//and a vflip is ^2.
	BAYER_ORDER_BGGR,
	BAYER_ORDER_GBRG,
	BAYER_ORDER_GRBG,
	BAYER_ORDER_RGGB
};

struct sensor_regs {
	uint16_t reg;
	uint16_t data;
};

struct mode_def
{
	struct sensor_regs *regs;
	int num_regs;
	int width;
	int height;
	MMAL_FOURCC_T encoding;
	enum bayer_order order;
	int native_bit_depth;
	uint8_t image_id;
	uint8_t data_lanes;
	int min_vts;
	int line_time_ns;
	uint32_t timing[5];
	uint32_t term[2];
	int black_level;
};

struct sensor_def
{
	char *name;
	struct mode_def *modes;
	int num_modes;
	struct sensor_regs *stop;
	int num_stop_regs;

	uint8_t i2c_addr;		// Device I2C slave address
	int i2c_addressing;		// Length of register address values
	int i2c_data_size;		// Length of register data to write

	//  Detecting the device
	int i2c_ident_length;		// Length of I2C ID register
	uint16_t i2c_ident_reg;		// ID register address
	uint16_t i2c_ident_value;	// ID register value

	// Flip configuration
	uint16_t vflip_reg;		// Register for VFlip
	int vflip_reg_bit;		// Bit in that register for VFlip
	uint16_t hflip_reg;		// Register for HFlip
	int hflip_reg_bit;		// Bit in that register for HFlip
	int flips_dont_change_bayer_order;	// Some sensors do not change the
						// Bayer order by adjusting X/Y starts
						// to compensate.

	uint16_t exposure_reg;
	int exposure_reg_num_bits;

	uint16_t vts_reg;
	int vts_reg_num_bits;

	uint16_t gain_reg;
	int gain_reg_num_bits;
};

#define NUM_ELEMENTS(a)  (sizeof(a) / sizeof(a[0]))

#include "ov5647_modes.h"
#include "imx219_modes.h"
#include "adv7282m_modes.h"

typedef struct {
	int mode;
	int hflip;
	int vflip;
	int exposure;
	int gain;
	char *output;
	int capture;
	int write_header;
	int timeout;
	int saverate;
	int bit_depth;
	int camera_num;
	int exposure_us;
	int i2c_bus;
	double awb_gains_r;
	double awb_gains_b;
	char *regs;
	int hinc;
	int vinc;
	double fps;
	int width;
	int height;
	int left;
	int top;
	char *write_header0;
	char *write_headerg;
	char *write_timestamps;
	int write_empty;
        int decodemetadata;
} RASPIRAW_PARAMS_T;

void start_camera_streaming(const struct sensor_def *sensor, struct mode_def *mode)
{
	int fd;

	vcos_log_error("Now streaming...");
}

void stop_camera_streaming(const struct sensor_def *sensor)
{
	int fd;
	fd = open(i2c_device_name, O_RDWR);
	if (!fd)
	{
		vcos_log_error("Couldn't open I2C device");
		return;
	}
	if (ioctl(fd, I2C_SLAVE_FORCE, sensor->i2c_addr) < 0)
	{
		vcos_log_error("Failed to set I2C address");
		return;
	}
	send_regs(fd, sensor, sensor->stop, sensor->num_stop_regs);
	close(fd);
}

void decodemetadataline(uint8_t *data, int bpp)
{
	int c=1;
	uint8_t tag,dta;
	uint16_t reg=-1;

	if (data[0]==0x0a)
	{

		while (data[c]!=0x07)
		{
			tag=data[c++];
			if (bpp==10 && (c%5)==4)
				c++;
			if (bpp==12 && (c%3)==2)
				c++;
			dta=data[c++];

			if (tag==0xaa)
				reg=(reg&0x00ff)|(dta<<8);
			else if (tag==0xa5)
				reg=(reg&0xff00)|dta;
			else if (tag==0x5a)
				vcos_log_error("Register 0x%04x = 0x%02x",reg++,dta);
			else if (tag==0x55)
				vcos_log_error("Skip     0x%04x",reg++);
			else
				vcos_log_error("Metadata decode failed %x %x %x",reg,tag,dta);
		}
	}
	else
		vcos_log_error("Doesn't looks like register set %x!=0x0a",data[0]);

}

int encoding_to_bpp(uint32_t encoding)
{
       switch(encoding)
       {
       case    MMAL_ENCODING_BAYER_SBGGR10P:
       case    MMAL_ENCODING_BAYER_SGBRG10P:
       case    MMAL_ENCODING_BAYER_SGRBG10P:
       case    MMAL_ENCODING_BAYER_SRGGB10P:
               return 10;
       case    MMAL_ENCODING_BAYER_SBGGR12P:
       case    MMAL_ENCODING_BAYER_SGBRG12P:
       case    MMAL_ENCODING_BAYER_SGRBG12P:
       case    MMAL_ENCODING_BAYER_SRGGB12P:
               return 12;
       default:
               return 8;
       };

}

int running = 0;
static void callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	static int count = 0;
	fprintf(stdout, "Buffer %p returned, filled %d, timestamp %llu, flags %04X", buffer, buffer->length, buffer->pts, buffer->flags);
	if (running) {
		RASPIRAW_PARAMS_T *cfg = (RASPIRAW_PARAMS_T *)port->userdata;

		if (!(buffer->flags&MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
		{
			// Save every Nth frame
			// SD card access is too slow to do much more.
			FILE *file;
			char filename[16];
			
			sprintf(&filename, "rxtest/rxpkt_%04d.bin", packet_idx)
			
			file = fopen(filename, "wb");
			if(file) {
				fwrite(buffer->data, buffer->length, 1, file);
				fclose(file);
			} else {
				fprintf(stdout, "File write error");
			}
			free(filename);
		}

		if (cfg->decodemetadata && (buffer->flags&MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
		{
			int bpp = encoding_to_bpp(port->format->encoding);
			vcos_log_error("First metadata line");
			decodemetadataline(buffer->data, bpp);
			vcos_log_error("Second metadata line");
			decodemetadataline(buffer->data+VCOS_ALIGN_UP(5*(port->format->es->video.width/4),16), bpp);
		}

		buffer->length = 0;
		mmal_port_send_buffer(port, buffer);
	} else {
		mmal_buffer_header_release(buffer);
	}
}

uint32_t order_and_bit_depth_to_encoding(enum bayer_order order, int bit_depth)
{
	//BAYER_ORDER_BGGR,
	//BAYER_ORDER_GBRG,
	//BAYER_ORDER_GRBG,
	//BAYER_ORDER_RGGB
	const uint32_t depth8[] = {
		MMAL_ENCODING_BAYER_SBGGR8,
		MMAL_ENCODING_BAYER_SGBRG8,
		MMAL_ENCODING_BAYER_SGRBG8,
		MMAL_ENCODING_BAYER_SRGGB8
	};
	const uint32_t depth10[] = {
		MMAL_ENCODING_BAYER_SBGGR10P,
		MMAL_ENCODING_BAYER_SGBRG10P,
		MMAL_ENCODING_BAYER_SGRBG10P,
		MMAL_ENCODING_BAYER_SRGGB10P
	};
	const uint32_t depth12[] = {
		MMAL_ENCODING_BAYER_SBGGR12P,
		MMAL_ENCODING_BAYER_SGBRG12P,
		MMAL_ENCODING_BAYER_SGRBG12P,
		MMAL_ENCODING_BAYER_SRGGB12P,
	};
	const uint32_t depth16[] = {
		MMAL_ENCODING_BAYER_SBGGR16,
		MMAL_ENCODING_BAYER_SGBRG16,
		MMAL_ENCODING_BAYER_SGRBG16,
		MMAL_ENCODING_BAYER_SRGGB16,
	};
	if (order < 0 || order > 3)
	{
		vcos_log_error("order out of range - %d", order);
		return 0;
	}

	switch(bit_depth)
	{
		case 8:
			return depth8[order];
		case 10:
			return depth10[order];
		case 12:
			return depth12[order];
		case 16:
			return depth16[order];
	}
	vcos_log_error("%d not one of the handled bit depths", bit_depth);
	return 0;
}

int main(int argc, char** argv) 
{
	RASPIRAW_PARAMS_T cfg = { 0 };
	uint32_t encoding;
	const struct sensor_def *sensor;
	struct mode_def *sensor_mode = NULL;
	unsigned int i;

	MMAL_COMPONENT_T *rawcam=NULL, *isp=NULL, *render=NULL;
	MMAL_STATUS_T status;
	MMAL_PORT_T *output = NULL;
	MMAL_POOL_T *pool = NULL;
	MMAL_CONNECTION_T *rawcam_isp = NULL;
	MMAL_CONNECTION_T *isp_render = NULL;
	MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_cfg;
	MMAL_PARAMETER_CAMERA_RX_TIMING_T rx_timing;
	
	bcm_host_init();
	vcos_log_register("SMVP_MMAL", VCOS_LOG_CATEGORY);

	fprintf(stdout, "\nMMAL Hacked Camera App %s\n\n", VERSION_STRING);

	sensor = &ov5647; // probe_sensor();
	sensor_mode = &sensor->modes[0];
	
	fprintf(stdout, "** Sensor Mode: %02x **\n", sensor_mode->image_id);

	cfg.bit_depth = sensor_mode->native_bit_depth;

	update_regs(sensor, sensor_mode, cfg.hflip, cfg.vflip, cfg.exposure, cfg.gain);
	
	encoding = order_and_bit_depth_to_encoding(sensor_mode->order, cfg.bit_depth);
	
	if (!encoding)
	{
		fprintf(stdout, "Failed to map bitdepth %d and order %d into encoding\n", cfg.bit_depth, sensor_mode->order);
		return -3;
	}
	
	fprintf(stdout, "Encoding %08X", encoding);

	bcm_host_init(); // is this needed 2x?
	vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);

	status = mmal_component_create("vc.ril.rawcam", &rawcam);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to create rawcam");
		return -1;
	}

	status = mmal_component_create("vc.ril.isp", &isp);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to create isp");
		goto component_destroy;
	}

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &render);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to create render");
		goto component_destroy;
	}

	output = rawcam->output[0];

	rx_cfg.hdr.id = MMAL_PARAMETER_CAMERA_RX_CONFIG;
	rx_cfg.hdr.size = sizeof(rx_cfg);
	status = mmal_port_parameter_get(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to get cfg");
		goto component_destroy;
	}
	if (sensor_mode->encoding || cfg.bit_depth == sensor_mode->native_bit_depth)
	{
		rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
		rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_NONE;
	}
	else
	{
		switch(sensor_mode->native_bit_depth)
		{
			case 8:
				rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_8;
				break;
			case 10:
				rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_10;
				break;
			case 12:
				rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_12;
				break;
			case 14:
				rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_16;
				break;
			case 16:
				rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_16;
				break;
			default:
				fprintf(stdout, "Unknown native bit depth %d", sensor_mode->native_bit_depth);
				rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
				break;
		}
		switch(cfg.bit_depth)
		{
			case 8:
				rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_8;
				break;
			case 10:
				rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_RAW10;
				break;
			case 12:
				rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_RAW12;
				break;
			case 14:
				rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_14;
				break;
			case 16:
				rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_16;
				break;
			default:
				fprintf(stdout, "Unknown output bit depth %d", cfg.bit_depth);
				rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
				break;
		}
	}
	vcos_log_error("Set pack to %d, unpack to %d", rx_cfg.unpack, rx_cfg.pack);
	if (sensor_mode->data_lanes)
		rx_cfg.data_lanes = sensor_mode->data_lanes;
	if (sensor_mode->image_id)
		rx_cfg.image_id = sensor_mode->image_id;
	status = mmal_port_parameter_set(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to set cfg");
		goto component_destroy;
	}

	rx_timing.hdr.id = MMAL_PARAMETER_CAMERA_RX_TIMING;
	rx_timing.hdr.size = sizeof(rx_timing);
	status = mmal_port_parameter_get(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to get timing");
		goto component_destroy;
	}
	if (sensor_mode->timing[0])
		rx_timing.timing1 = sensor_mode->timing[0];
	if (sensor_mode->timing[1])
		rx_timing.timing2 = sensor_mode->timing[1];
	if (sensor_mode->timing[2])
		rx_timing.timing3 = sensor_mode->timing[2];
	if (sensor_mode->timing[3])
		rx_timing.timing4 = sensor_mode->timing[3];
	if (sensor_mode->timing[4])
		rx_timing.timing5 = sensor_mode->timing[4];
	if (sensor_mode->term[0])
		rx_timing.term1 = sensor_mode->term[0];
	if (sensor_mode->term[1])
		rx_timing.term2 = sensor_mode->term[1];
	fprintf(stdout, "Timing %u/%u, %u/%u/%u, %u/%u",
		rx_timing.timing1, rx_timing.timing2,
		rx_timing.timing3, rx_timing.timing4, rx_timing.timing5,
		rx_timing.term1,  rx_timing.term2);
	status = mmal_port_parameter_set(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to set timing");
		goto component_destroy;
	}

	if (cfg.camera_num != -1) {
		fprintf(stdout, "Set camera_num to %d", cfg.camera_num);
		status = mmal_port_parameter_set_int32(output, MMAL_PARAMETER_CAMERA_NUM, cfg.camera_num);
		if (status != MMAL_SUCCESS)
		{
			fprintf(stdout, "Failed to set camera_num");
			goto component_destroy;
		}
	}

	status = mmal_component_enable(rawcam);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to enable rawcam");
		goto component_destroy;
	}
	status = mmal_component_enable(isp);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to enable isp");
		goto component_destroy;
	}
	status = mmal_component_enable(render);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to enable render");
		goto component_destroy;
	}

	output->format->es->video.crop.width = sensor_mode->width;
	output->format->es->video.crop.height = sensor_mode->height;
	output->format->es->video.width = VCOS_ALIGN_UP(sensor_mode->width, 16);
	output->format->es->video.height = VCOS_ALIGN_UP(sensor_mode->height, 16);
	output->format->encoding = encoding;

	status = mmal_port_format_commit(output);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed port_format_commit");
		goto component_disable;
	}

	output->buffer_size = output->buffer_size_recommended;
	output->buffer_num = output->buffer_num_recommended;

	status = mmal_port_parameter_set_boolean(output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to set zero copy");
		goto component_disable;
	}

	fprintf(stdout, "Create pool of %d buffers of size %d", output->buffer_num, output->buffer_size);
	pool = mmal_port_pool_create(output, output->buffer_num, output->buffer_size);
	if (!pool)
	{
		fprintf(stdout, "Failed to create pool");
		goto component_disable;
	}

	output->userdata = (struct MMAL_PORT_USERDATA_T *)&cfg;
	status = mmal_port_enable(output, callback);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to enable port");
		goto pool_destroy;
	}
	running = 1;
	for(i = 0; i<output->buffer_num; i++)
	{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);

		if (!buffer)
		{
			fprintf(stdout, "Where'd my buffer go?!");
			goto port_disable;
		}
		status = mmal_port_send_buffer(output, buffer);
		if (status != MMAL_SUCCESS)
		{
			fprintf(stdout, "mmal_port_send_buffer failed on buffer %p, status %d", buffer, status);
			goto port_disable;
		}
		fprintf(stdout, "Sent buffer %p", buffer);
	}
		
	start_camera_streaming(sensor, sensor_mode);

	vcos_sleep(10000000);
	running = 0;

	stop_camera_streaming(sensor);

port_disable:
	if (cfg.capture)
	{
		status = mmal_port_disable(output);
		if (status != MMAL_SUCCESS)
		{
			fprintf(stdout, "Failed to disable port");
			return -1;
		}
	}
pool_destroy:
	if (pool)
		mmal_port_pool_destroy(output, pool);
	if (isp_render)
	{
		mmal_connection_disable(isp_render);
		mmal_connection_destroy(isp_render);
	}
	if (rawcam_isp)
	{
		mmal_connection_disable(rawcam_isp);
		mmal_connection_destroy(rawcam_isp);
	}
component_disable:
	if (brcm_header)
		free(brcm_header);
	status = mmal_component_disable(render);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to disable render");
	}
	status = mmal_component_disable(isp);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to disable isp");
	}
	status = mmal_component_disable(rawcam);
	if (status != MMAL_SUCCESS)
	{
		fprintf(stdout, "Failed to disable rawcam");
	}
component_destroy:
	if (rawcam)
		mmal_component_destroy(rawcam);
	if (isp)
		mmal_component_destroy(isp);
	if (render)
		mmal_component_destroy(render);

	if (cfg.write_timestamps)
	{
		// Save timestamps
		FILE *file;
		file = fopen(cfg.write_timestamps, "wb");
		if (file)
		{
			int64_t old = 0;
			PTS_NODE_T aux;
			for(aux = cfg.ptsa; aux != cfg.ptso; aux = aux->nxt)
			{
				if (aux == cfg.ptsa)
				{
					fprintf(file, ",%d,%lld\n", aux->idx, aux->pts);
				}
				else
				{
					fprintf(file, "%lld,%d,%lld\n", aux->pts-old, aux->idx, aux->pts);
				}
				old = aux->pts;
			}
			fclose(file);
		}

		while (cfg.ptsa != cfg.ptso)
		{
			PTS_NODE_T aux = cfg.ptsa->nxt;
			free(cfg.ptsa);
			cfg.ptsa = aux;
		}
		free(cfg.ptso);
	}

	return 0;
}
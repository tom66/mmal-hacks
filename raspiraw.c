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
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h> /* usleep */
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include "interface/vcos/vcos.h"
#include "bcm_host.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_connection.h"
#include "raw_header.h"

//#include "EGL/eglext.h"
#include <epoxy/gl.h>
#include <epoxy/glx.h>
#include <epoxy/egl.h>
//#include <GL/glew.h>

//#include <GL/glut.h>
#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_EGL

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#include <GL/glu.h> //gluGetError
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <EGL/eglext_brcm.h>

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_connection.h"

#include "shader_utils.h"

int packet_idx = 0;

struct brcm_raw_header *brcm_header = NULL;

GLuint program;
GLint attribute_coord1d;
GLint uniform_offset_x;
GLint uniform_scale_x;
GLint uniform_mytexture;
GLint uniform_wavenum;
GLint uniform_NWAVES;
GLint uniform_nxtiles;
GLint uniform_xtile;
GLint uniform_color;

bool interpolate = false;
bool clamp = true;
bool showpoints = false;

#define WAVE_SIZE 		16384				// How big each acquisition is
#define NPOINTS 		2048				// How many points to render
#define TEXSIZE 		1024				// Size of the textures
#define NTEXTURES 		(NPOINTS/TEXSIZE)	// Number of textures
#define NWAVES 			6					// Number of waves to render per frame

GLuint vbo;
GLbyte graph[NTEXTURES][TEXSIZE * NWAVES];
//GLbyte graph[NPOINTS * NWAVES];
GLuint texture_id[NTEXTURES];

float offset_x = 0.0;
float scale_x = 1.0;
GLuint cam_ytex, cam_utex, cam_vtex;
EGLImageKHR yimg = EGL_NO_IMAGE_KHR;
EGLImageKHR uimg = EGL_NO_IMAGE_KHR;
EGLImageKHR vimg = EGL_NO_IMAGE_KHR;

EGLDisplay GDisplay;

GLFWwindow* win;

MMAL_BUFFER_HEADER_T shared_buf;
bool_t got_frame = 0, aborted = 0;
VCOS_MUTEX_T mutex;

unsigned long int frame_time_usec = 0;

unsigned int fps_frames = 0;
struct timeval fps_start = {0,0};

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

static void check_real(const char *file, int line) {
	int q;
	do {
		q=glGetError();
		if (q) {
			printf("%s:%d: glError: %s (%x)\n", file, line, gluErrorString(q), q);
		}
	} while (q);
}

#define check() check_real(__FILE__, __LINE__)
#define C(...) __VA_ARGS__;check();

int initgraph() 
{
	glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
	if (!glfwInit()) {
		fprintf(stderr, "glfw init failed\n");
		return 1;
	}
		
 	if (!(win=glfwCreateWindow(1280, 768, "OscilloTest", NULL, NULL))) {
		fprintf(stderr, "window create failed\n");
		return 1;
	}
	glfwMakeContextCurrent(win);
	GDisplay = glfwGetEGLDisplay();

	C(glEnable(GL_TEXTURE_EXTERNAL_OES));

	printf ("win=%p tid=%d\n",win, syscall(SYS_gettid));

	GLint max_units;

	C(glGetIntegerv(GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, &max_units));
	if (max_units < 1) {
		fprintf(stderr, "Your GPU does not have any vertex texture image units\n");
		return 1;
	}
	
	C(glGetIntegerv(GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, &max_units));
	if (max_units < 1) {
		fprintf(stderr, "Your GPU does not have any vertex texture image units\n");
		return 1;
	}

	GLfloat range[2];

	glGetFloatv(GL_ALIASED_POINT_SIZE_RANGE, range);
	if (range[1] < 5.0) {
		fprintf(stderr, "WARNING: point sprite range (%f, %f) too small\n", range[0], range[1]);
	}

	if (!init_resources()) {
		fprintf(stderr, "Error in init_resources");
		return 1;
	}
	
	assert(!glGetError());
	
	return 0;
}

int init_resources() 
{
	program = create_program("graph.v.glsl", "graph.f.glsl");
	//program = create_program("graph.v.glsl", "antialias.f.glsl");
	
	if (program == 0) {
		fprintf(stderr, "Can't create program");
		return 0;
	}
	
	printf("texture_id=%p\n",texture_id);

	attribute_coord1d = get_attrib(program, "coord1d");
	uniform_offset_x = get_uniform(program, "offset_x");
	uniform_scale_x = get_uniform(program, "scale_x");
	uniform_mytexture = get_uniform(program, "mytexture");
	uniform_wavenum = get_uniform(program, "wavenum");
	uniform_NWAVES = get_uniform(program, "NWAVES");
	uniform_nxtiles = get_uniform(program, "nxtiles");
	uniform_xtile = get_uniform(program, "xtile");
	uniform_color = get_uniform(program, "color");

	if (attribute_coord1d == -1 || uniform_offset_x == -1 || uniform_scale_x == -1 || uniform_mytexture == -1) {
		fprintf(stderr, "Bounds checking error");
		return 0;
	}

	/* Upload the texture with our datapoints */
	glActiveTexture(GL_TEXTURE0);
	
	glGenTextures(1, &cam_ytex);
	glGenTextures(NWAVES * NTEXTURES, texture_id);

	for (int i=0; i < (NWAVES * NTEXTURES); i++) {
		printf("texture %d has id %d\n", i, texture_id[i]);
		glBindTexture(GL_TEXTURE_2D, texture_id[i]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, TEXSIZE, 1, 0, GL_LUMINANCE, GL_BYTE, graph[0]);
		
		/* Set texture wrapping mode */
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT);
		
		/* Set texture interpolation mode */
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, interpolate ? GL_LINEAR : GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, interpolate ? GL_LINEAR : GL_NEAREST);
	}

	// Create the vertex buffer object
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE,GL_ONE);
	// Create an array with only x values.
	GLfloat line[NPOINTS];

	// Fill it in just like an array
	for (int i = 0; i < NPOINTS; i++) {
		line[i] = (i - ((NPOINTS-1)/2.0)) / ((NPOINTS-1)/2.0);
	}

	// Tell OpenGL to copy our array to the buffer object
	glBufferData(GL_ARRAY_BUFFER, sizeof line, line, GL_STATIC_DRAW);

	// Enable point size control in vertex shader
#ifndef GL_ES_VERSION_2_0
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
#endif

	return 1;
}

void graph_set_buffer(MMAL_BUFFER_HEADER_T *buf) 
{
	//printf("cb: %08x %d tid=%d\n", buf->data, buf->length, syscall(SYS_gettid));
	
	/*
	{
		FILE *f=fopen("/tmp/q","wb");
		fwrite(buf->data, buf->length, 1, f);
		fclose(f);
	}
	*/
	
	assert(!glGetError());
	
#if 0
	C(glBindTexture(GL_TEXTURE_EXTERNAL_OES, cam_ytex));

	if(yimg != EGL_NO_IMAGE_KHR){
		eglDestroyImageKHR(GDisplay, yimg);
		yimg = EGL_NO_IMAGE_KHR;
	}

	yimg = eglCreateImageKHR(GDisplay, 
			EGL_NO_CONTEXT, 
			/*EGL_IMAGE_BRCM_RAW_PIXELS,*/ EGL_IMAGE_BRCM_MULTIMEDIA_Y, 
			(EGLClientBuffer) buf->data, 
			NULL);
	check();
	glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, yimg);
	check();
#else
	int i = 0;
	int w = 0;

	for (int w=0; w < NWAVES; w++) {
		for (int i=0; i < NTEXTURES; i++) {
			//void *ptr = (int8_t *)((buf->data + (262144 / 2) - TEXSIZE) + (i * TEXSIZE)); // + i * TEXSIZE * NWAVES;
			void *ptr = (int8_t *)(buf->data + (i * TEXSIZE) + (WAVE_SIZE * (w + 1)) - (WAVE_SIZE / 2) - TEXSIZE);
			
			// WAVE_SIZE
			
			glBindTexture(GL_TEXTURE_2D, texture_id[i + (w * NTEXTURES)]);
			check();
			glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, TEXSIZE, 1, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, ptr);
			check();
		}
	}
	
	/* Set texture wrapping mode */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT);
	check();

	/* Set texture interpolation mode */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, interpolate ? GL_LINEAR : GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, interpolate ? GL_LINEAR : GL_NEAREST);
	check();
#endif
}

// this literally does nothing useful
void start_camera_streaming(const struct sensor_def *sensor, struct mode_def *mode)
{
	printf("Now streaming...\n");
}

// this too does nothing useful
void stop_camera_streaming(const struct sensor_def *sensor)
{
	printf("...and we're out\n");
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

void graph_display() 
{
	struct timeval now, delta;

	assert(!glGetError());
	
	// Sum frame times together and print estimated fps every hundred frames
	gettimeofday (&now, NULL);
	timersub (&now, &fps_start, &delta);
	
	frame_time_usec += (delta.tv_sec * 1000000) + delta.tv_usec;
	fps_frames++;
	
	if(fps_frames >= 100) {
		frame_time_usec /= 100;
		printf("FPS: %.4f (frame time = ~%d us)\n", 1e6 / frame_time_usec, frame_time_usec);
		
		fps_frames = 0;
		frame_time_usec = 0;
	}
	
	fps_start = now;

	check();
	
	glUseProgram(program);
	check();
	
	glUniform1i(uniform_mytexture, 0);
	check();
	glUniform1f(uniform_offset_x, offset_x);
	check();
	glUniform1f(uniform_scale_x, scale_x);
	check();
	glUniform1f(uniform_NWAVES, 0);
	check();

	glClearColor(0.0, 0.0, 0.0, 0.0);
	check();
	glClear(GL_COLOR_BUFFER_BIT);
	check();

	/* Draw using the vertices in our vertex buffer object */
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	check();

	glEnableVertexAttribArray(attribute_coord1d);
	check();
	glVertexAttribPointer(attribute_coord1d, 1, GL_FLOAT, GL_FALSE, 0, 0);
	check();

	float scale = 0.0625f; // 16./NWAVES/2;
	
	//glUniform1f(uniform_wavenum, 0 /*(2*i+1.0)/(2*NWAVES)*/);
	//check();
	glUniform1f(uniform_nxtiles, NTEXTURES);
	check();
	glUniform4f(uniform_color, 1*scale,4*scale,1*scale,1);
	check();
			
	for (int j=0; j<NTEXTURES; j++) {
		//int j=1;
		for (int i=0; i<NWAVES; i++) {
			glBindTexture(GL_TEXTURE_2D, texture_id[(i * NTEXTURES) + j]);
			check();
			
			glUniform1f(uniform_xtile, ((float)j*2-NTEXTURES+1)/NTEXTURES);
			check();
			
			//glUniform1f(uniform_offset_x, offset_x);
			//glUniform1f(uniform_scale_x, scale_x);
			
			
			//			glUniform4f(uniform_color, (j&1)*scale,((j&2)>>1)*scale,((j&4)>>2)*scale,1);
			//glUniform4f(uniform_color, (i&1)*scale,((i&2)>>1)*scale,((i&4)>>2)*scale,1);

			//glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, NPOINTS, NWAVES, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, graph+NPOINTS*i);
			glDrawArrays(GL_LINE_STRIP, 0, NPOINTS);
			check();
		}
		//		glutSwapBuffers();	usleep(50000);
	}
	
	assert(!glGetError());
	glfwSwapBuffers(win);
	//glutSwapBuffers();
	//glutPostRedisplay();
}

int running = 0;

static void callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	static int count = 0;
	//printf("Buffer %p returned, data %p, filled %d, offset %d, timestamp %llu, flags %04X, running %d\n", buffer, buffer->data, buffer->length, buffer->offset, buffer->pts, buffer->flags, running);

	FILE *file;
	char filename[32];
		
	//RASPIRAW_PARAMS_T *cfg = (RASPIRAW_PARAMS_T *)port->userdata;

#if 1
	if (!(buffer->flags&MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
	{
		//sprintf(filename, "rxtest/rxpkt_%04d.bin", packet_idx);
		//printf("Filename: %s\n", filename);
			
		while (vcos_mutex_lock(&mutex) != VCOS_SUCCESS);
		shared_buf = *buffer;
		//printf ("shared_buf: %p -> %p %d\n", &shared_buf, shared_buf.data, shared_buf.length);
		got_frame = 1;
		vcos_mutex_unlock(&mutex);
			
		packet_idx++;
	}

	/*
	if ((buffer->flags&MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
	{
		printf("Got a metadata packet, maybe I shall do something with it sometime\n");
	}
	*/
#endif

	//printf("end of BufferTest...\n");
	buffer->length = 0;
	mmal_buffer_header_release(buffer);
	mmal_port_send_buffer(port, buffer);
	//printf("end of callback...\n");
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
		printf("order out of range - %d\n", order);
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
	printf("%d not one of the handled bit depths\n", bit_depth);
	return 0;
}

void signal_abort(int i) 
{
	aborted = 1;
	write(2, "aborting\n", 9);
}

int main(int argc, char** argv) 
{
	RASPIRAW_PARAMS_T cfg = { 0 };
	uint32_t encoding;
	const struct sensor_def *sensor;
	struct mode_def *sensor_mode = NULL;
	unsigned int i;
	unsigned int camera_num = 1;
	uint32_t frame_counter = 0;

	MMAL_COMPONENT_T *rawcam=NULL, *isp=NULL, *render=NULL;
	MMAL_STATUS_T status;
	MMAL_PORT_T *output = NULL;
	MMAL_POOL_T *pool = NULL;
	MMAL_CONNECTION_T *rawcam_isp = NULL;
	MMAL_CONNECTION_T *isp_render = NULL;
	MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_cfg;
	MMAL_PARAMETER_CAMERA_RX_TIMING_T rx_timing;
	
	signal(SIGINT, signal_abort);
	signal(SIGQUIT, signal_abort);
	
	initgraph();
	bcm_host_init();
	vcos_log_register("SMVP_MMAL", VCOS_LOG_CATEGORY);

	printf("\nMMAL Hacked Camera App %s\n\n", VERSION_STRING);

	sensor = &ov5647; // probe_sensor();
	sensor_mode = &sensor->modes[0];
	
	printf("** Sensor Mode: %02x **\n", sensor_mode->image_id);

	cfg.bit_depth = sensor_mode->native_bit_depth;
	encoding = order_and_bit_depth_to_encoding(sensor_mode->order, cfg.bit_depth);
	
	if (!encoding)
	{
		printf("Failed to map bitdepth %d and order %d into encoding\n", cfg.bit_depth, sensor_mode->order);
		return -3;
	}
	
	printf("Encoding %08X\n", encoding);

	bcm_host_init(); // is this needed 2x?
	vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);

	status = mmal_component_create("vc.ril.rawcam", &rawcam);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to create rawcam\n");
		return -1;
	}

	status = mmal_component_create("vc.ril.isp", &isp);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to create isp\n");
		goto component_destroy;
	}

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &render);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to create render\n");
		goto component_destroy;
	}

	output = rawcam->output[0];

	rx_cfg.hdr.id = MMAL_PARAMETER_CAMERA_RX_CONFIG;
	rx_cfg.hdr.size = sizeof(rx_cfg);
	status = mmal_port_parameter_get(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to get cfg\n");
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
				printf("Unknown native bit depth %d\n", sensor_mode->native_bit_depth);
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
				printf("Unknown output bit depth %d\n", cfg.bit_depth);
				rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
				break;
		}
	}
	printf("Set pack to %d, unpack to %d\n", rx_cfg.unpack, rx_cfg.pack);
	if (sensor_mode->data_lanes)
		rx_cfg.data_lanes = sensor_mode->data_lanes;
	if (sensor_mode->image_id)
		rx_cfg.image_id = sensor_mode->image_id;
	
	printf("Set data_lanes to %d, image_id to 0x%02x\n", rx_cfg.data_lanes, rx_cfg.image_id);
	
	status = mmal_port_parameter_set(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to set cfg\n");
		goto component_destroy;
	}

	rx_timing.hdr.id = MMAL_PARAMETER_CAMERA_RX_TIMING;
	rx_timing.hdr.size = sizeof(rx_timing);
	
	status = mmal_port_parameter_get(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to get timing\n");
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
	printf("Timing %u/%u, %u/%u/%u, %u/%u\n",
		rx_timing.timing1, rx_timing.timing2,
		rx_timing.timing3, rx_timing.timing4, rx_timing.timing5,
		rx_timing.term1,  rx_timing.term2);
	status = mmal_port_parameter_set(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to set timing\n");
		goto component_destroy;
	}

	printf("Set camera_num to %d\n", camera_num);
	status = mmal_port_parameter_set_int32(output, MMAL_PARAMETER_CAMERA_NUM, camera_num);
	
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to set camera_num\n");
		goto component_destroy;
	}
	
	status = mmal_component_enable(rawcam);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to enable rawcam\n");
		goto component_destroy;
	}
	status = mmal_component_enable(isp);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to enable isp\n");
		goto component_destroy;
	}
	status = mmal_component_enable(render);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to enable render\n");
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
		printf("Failed port_format_commit\n");
		goto component_disable;
	}

	output->buffer_size = output->buffer_size_recommended;
	output->buffer_num = output->buffer_num_recommended;

	status = mmal_port_parameter_set_boolean(output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to set zero copy\n");
		goto component_disable;
	}

	printf("Create pool of %d buffers of size %d\n", output->buffer_num, output->buffer_size);
	pool = mmal_port_pool_create(output, output->buffer_num, output->buffer_size);
	if (!pool)
	{
		printf("Failed to create pool\n");
		goto component_disable;
	}

	output->userdata = (struct MMAL_PORT_USERDATA_T *)&cfg;
	status = mmal_port_enable(output, callback);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to enable port\n");
		goto pool_destroy;
	}
	running = 1;
	for(i = 0; i<output->buffer_num; i++)
	{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);

		if (!buffer)
		{
			printf("Where'd my buffer go?!\n");
			goto port_disable;
		}
		status = mmal_port_send_buffer(output, buffer);
		if (status != MMAL_SUCCESS)
		{
			printf("mmal_port_send_buffer failed on buffer %p, status %d\n", buffer, status);
			goto port_disable;
		}
		printf("Sent buffer %p\n", buffer);
	}
		
	start_camera_streaming(sensor, sensor_mode);
	frame_counter = 0;
	
	do {
		while (vcos_mutex_lock(&mutex) != VCOS_SUCCESS);
		if (got_frame) {
			graph_set_buffer(&shared_buf);
			got_frame = 0;
			vcos_mutex_unlock(&mutex);
			//printf("got frame %d\n", frame_counter++);
			graph_display();
		} else {
			vcos_mutex_unlock(&mutex);
		}
		//usleep (1000000.0/cfg.fps);
		//usleep (1000);
	} while (!aborted);
	
	running = 0;

	stop_camera_streaming(sensor);

port_disable:
	if (cfg.capture)
	{
		status = mmal_port_disable(output);
		if (status != MMAL_SUCCESS)
		{
			printf("Failed to disable port\n");
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
		printf("Failed to disable render\n");
	}
	status = mmal_component_disable(isp);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to disable isp\n");
	}
	status = mmal_component_disable(rawcam);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to disable rawcam\n");
	}
component_destroy:
	if (rawcam)
		mmal_component_destroy(rawcam);
	if (isp)
		mmal_component_destroy(isp);
	if (render)
		mmal_component_destroy(render);

	return 0;
}
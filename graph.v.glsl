attribute float coord1d;
varying vec4 f_color;
uniform float offset_x;
uniform float scale_x;
uniform float wavenum;
uniform float nwaves;
uniform float nxtiles;
uniform float xtile;
uniform sampler2D mytexture;
uniform vec4 color;

void main(void) {
	float x = (coord1d / scale_x) - offset_x;
	float y = (texture2D(mytexture, vec2(x  / 2.0 + 0.5, wavenum)).r - 0.0) * 2.0;  // was .r - 0.5

	gl_Position = vec4(coord1d/nxtiles + xtile, y, 0.0, 1.0);
	f_color = color;
	gl_PointSize = 2.0;
}

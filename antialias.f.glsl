#version 120
uniform vec3 color;

void main(void) {
    float r = 0.0, delta = 0.0, alpha = 1.0;
    vec2 cxy = 2.0 * gl_PointCoord - 1.0;
    vec2 pc = gl_PointCoord;
    gl_FragColor = vec4(color, (1-abs(cxy.x)) * (1-abs(cxy.y)));
}

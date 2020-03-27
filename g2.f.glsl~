#version 120
uniform vec3 color;

void main(void) {
    float r = 0.0, delta = 0.0, alpha = 1.0;
    vec2 cxy = 2.0 * gl_PointCoord - 1.0;
    vec2 pc = gl_PointCoord;
    //r = dot(cxy, cxy);
    //vec4 px[4] = [ (1.0, 0.0, 0.0, 1.0),
    //               (0.0, 1.0, 0.0, 1.0),
    //               (1.0, 1.0, 0.0, 1.0),
    //               (0.0, 0.0, 1.0, 1.0)];
                   
    //alpha = pc.x * pc.x * pc.y * pc.y;
    //gl_FragColor = color * (alpha);
#if 0
    float tl = pc.s     * (1-pc.t),
      tr = (1-pc.s) * (1-pc.t),
      bl = pc.s     * pc.t,
      br = (1-pc.s) * pc.t;

    //gl_FragColor = (pc.s<=.51) ? ((pc.t<=.5) ? vec4(1,0,0,1) : vec4(0,1,0,1)) :
    //        ((pc.t<=.5) ? vec4(1,1,0,1) : vec4(0,0,1,1));
    gl_FragColor = color * ((pc.s<=.51) ? ((pc.t<=.5) ? bl : tl) :
                            ((pc.t<=.5) ? br : tr));
#else
    //gl_FragColor = color * (1-abs(cxy.x)) * (1-abs(cxy.y));
    gl_FragColor = vec4(color, (1-abs(cxy.x)) * (1-abs(cxy.y)));
#endif
}

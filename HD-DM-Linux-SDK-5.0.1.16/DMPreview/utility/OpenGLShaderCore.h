#ifndef OPENGLSHADERCORE_H
#define OPENGLSHADERCORE_H

#define GLSL(version, kernel) "#version " version "\n" #kernel

#if defined(Q_PROCESSOR_ARM)
#define GLSL_VERSION "300 es"
#else
#define GLSL_VERSION "440"
#endif

static const char *shaderVert = GLSL(
GLSL_VERSION,
#ifdef GL_ES
precision mediump float;
#endif
uniform mat4 mvp_matrix;
in vec4 a_position;
in vec3 a_color;
out vec4 v_color;
void main()
{
    gl_Position = mvp_matrix * a_position;
    v_color = vec4(a_color, 1.0);
#ifdef GL_ES
    gl_PointSize = 2;
#endif
}
);

static const char *shaderFragment = GLSL(
GLSL_VERSION,
#ifdef GL_ES
precision mediump float;
#endif
uniform bool bSingleColor;
in mediump vec4 v_color;
out mediump vec4 out_Color;
void main()
{
    if(bSingleColor){
        out_Color = vec4(0.0, 1.0, 0.0, 1.0);
    }else{
        out_Color = v_color;
    }
}
);

static const char *imuShaderVert = GLSL(
GLSL_VERSION,
#ifdef GL_ES
precision mediump float;
#endif
uniform mat4 mvp_matrix;
in vec4 a_position;
in vec4 a_color;
out vec4 v_color;
void main()
{
    gl_Position = mvp_matrix * a_position;
    v_color = a_color;
}
);

static const char *imuShaderFragment = GLSL(
GLSL_VERSION,
#ifdef GL_ES
precision mediump float;
#endif
in mediump vec4 v_color;
out mediump vec4 out_Color;
void main()
{
    out_Color = v_color;
}
);


#endif // OPENGLSHADERCORE_H

#version 300 es

precision mediump float;

in vec4 vColor;
in vec2 vTexCoord;

out vec4 fColor;


uniform sampler2D brickTex;
uniform sampler2D smokeTex;

void
main()
{
    vec4 t0 = texture2D(brickTex, vTexCoord);
    vec4 t1 = texture2D(smokeTex, vTexCoord);
    gl_fragColor = vColor * mix(texture2D(brickTex, vTexCoord), texture2D(smokeTex, vTexCoord), 0.5);
}
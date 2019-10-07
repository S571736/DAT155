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
    //vec4 t0 = texture(brickTex, vTexCoord.xy);
    //vec4 t1 = texture(smokeTex, vTexCoord.xy);
    fColor = vColor * mix(texture(brickTex, vTexCoord.xy), texture(smokeTex, vTexCoord.xy), 0.5);

}
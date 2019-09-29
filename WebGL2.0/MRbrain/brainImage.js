"use strict";

var texSize = 256;
var slice = 0;
var sliceData = new Uint8Array(texSize*texSize);
var textureId = new Array(108);
/*
for(var i = 0; i<texSize; i++) for(var j=0; j<texSize; j++) {
  sliceData[i*texSize+j] = data[slice*texSize*texSize + i*texSize +j];
}
*/


var canvas;
var gl;

var program;

var pointsArray = [];
var texCoordsArray = [];

window.onload = function init() {
    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }


    gl.viewport( 0, 0, canvas.width, canvas.height );
    gl.clearColor( 1.0, 1.0, 1.0, 1.0 );

    //
    //  Load shaders and initialize attribute buffers
    //
    program = initShaders( gl, "vertex-shader", "fragment-shader" );
    gl.useProgram( program );

    pointsArray.push(vec2(-1, -1));
    pointsArray.push(vec2(-1, 1));
    pointsArray.push(vec2(1, 1));
    pointsArray.push(vec2(1, -1));

    texCoordsArray.push(vec2(0, 0));
    texCoordsArray.push(vec2(0, 1));
    texCoordsArray.push(vec2(1, 1));
    texCoordsArray.push(vec2(1, 0));

    var tBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(texCoordsArray), gl.STATIC_DRAW);

    var texCoordLoc = gl.getAttribLocation( program, "aTexCoord");
    gl.vertexAttribPointer(texCoordLoc, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(texCoordLoc);

    var vBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation( program, "aPosition");
    gl.vertexAttribPointer(positionLoc, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray( positionLoc );

    //var textureId = gl.createTexture();
    //gl.activeTexture(gl.TEXTURE0);
    //gl.bindTexture(gl.TEXTURE_2D, textureId);

    document.getElementById("slider").onchange = function(event) {
        slice = event.target.value;
        gl.bindTexture(gl.TEXTURE_2D, textureId[slice]);
        render();
    };


for(var k=0; k<108; k++) {
  //var k = 107;
  textureId[k] = gl.createTexture();
  gl.activeTexture(gl.TEXTURE0);
  gl.bindTexture(gl.TEXTURE_2D, textureId[k]);
  for(var i = 0; i<texSize; i++) for(var j=0; j<texSize; j++)
    //sliceData[i*texSize+j] = data[k*texSize*texSize + i*texSize +j];
    sliceData[i*texSize+j] = 255 - data[k*texSize*texSize + i*texSize +j];
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.LUMINANCE, texSize, texSize, 0,
  gl.LUMINANCE, gl.UNSIGNED_BYTE, sliceData);
  gl.generateMipmap( gl.TEXTURE_2D );
  gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
  gl.NEAREST_MIPMAP_LINEAR );
  gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST );
  };

  gl.bindTexture(gl.TEXTURE_2D, textureId[0]);
  render();

function render() {
    gl.clear( gl.COLOR_BUFFER_BIT );
    gl.drawArrays( gl.TRIANGLE_FAN, 0, 4 );
}
};

"use strict";

var canvas;
var gl;

var t1, t2;
t1 = new Date();

// turtle points

var size = 0.2;

var distance = 0.005;
var speed = 0.01;

var buffer1, buffer2, buffer3

var numSprites = 100000;
var spriteArray = [];
var directionArray = new Float32Array(numSprites);

for(var i = 0; i<numSprites; i++) {
  var theta = i*2*Math.PI/numSprites;
  spriteArray[i] = vec2(Math.cos(theta), Math.sin(theta));
  directionArray[i] =theta+0.1*Math.random();
}

var pointsArray = [
    vec2(size, 0.0),
    vec2(-size, size * 0.8 ),
    vec2(-size * 0.4, 0) ,
    vec2(-size * 0.4, 0) ,
    vec2(-size, -size * 0.8),
    vec2(size, 0.0),
];

var program1, program2;
var texture1;

var framebuffer, renderbuffer;


window.onload = function init() {
    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }


// Create an empty texture

    texture1 = gl.createTexture();
    gl.activeTexture( gl.TEXTURE0 );
    gl.bindTexture( gl.TEXTURE_2D, texture1 );
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, 64, 64, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST_MIPMAP_LINEAR );
    gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST );

    gl.bindTexture(gl.TEXTURE_2D, null);


// Allocate a framebuffer object

   framebuffer = gl.createFramebuffer();
   gl.bindFramebuffer( gl.FRAMEBUFFER, framebuffer);

   renderbuffer = gl.createRenderbuffer();
   gl.bindRenderbuffer(gl.RENDERBUFFER, renderbuffer);

// Attach color buffer


   gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, texture1, 0);


// check for completeness

   var status = gl.checkFramebufferStatus(gl.FRAMEBUFFER);
   if(status != gl.FRAMEBUFFER_COMPLETE) alert('Frame Buffer Not Complete');

    //
    //  Load shaders and initialize attribute buffers
    //
    program1 = initShaders( gl, "vertex-shader1", "fragment-shader1" );
    program2 = initShaders( gl, "vertex-shader2", "fragment-shader2" );

    gl.useProgram( program1 );


    // Create and initialize a buffer object with turtle vertices

    buffer1 = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, buffer1 );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW );

    // Initialize the vertex position attribute from the vertex shader

    var positionLoc = gl.getAttribLocation( program1, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 2, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( positionLoc );

    // Render  sprite

    gl.viewport(0, 0, 64, 64);
    gl.clearColor(1.0, 0.0, 0.0, 0.0);
    gl.clear(gl.COLOR_BUFFER_BIT );

    gl.drawArrays(gl.TRIANGLES, 0, 6);


    // Bind to window system frame buffer, unbind the texture

    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.bindRenderbuffer(gl.RENDERBUFFER, null);


    gl.disableVertexAttribArray(positionLoc);

    gl.useProgram(program2);

    gl.activeTexture(gl.TEXTURE0);

    gl.bindTexture(gl.TEXTURE_2D, texture1);

    // send data to GPU for normal render

    //  pointsArray


    buffer2 = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, buffer2);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(spriteArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation( program2, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 2, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( positionLoc );

    buffer3 = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, buffer3 );
    gl.bufferData( gl.ARRAY_BUFFER, directionArray, gl.STATIC_DRAW );

    var directionLoc = gl.getAttribLocation( program2, "aDirection" );
    gl.vertexAttribPointer( directionLoc, 1, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( directionLoc );

    gl.uniform1i( gl.getUniformLocation(program2, "textureMap"), 0);
    gl.clearColor( 0.0, 1.0, 1.0, 1.0 );

    gl.viewport(0, 0, 1024, 1024);


    render();
}


function render() {

    gl.clear( gl.COLOR_BUFFER_BIT );

    // render point with texture

    for(var i=0; i<numSprites; i++) {
      directionArray[i] += .1*(Math.random()-0.5);
      spriteArray[i] = add(spriteArray[i], vec2(Math.cos(directionArray[i])*distance,Math.sin(directionArray[i])* distance));
    };

    gl.bindBuffer( gl.ARRAY_BUFFER, buffer2);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(spriteArray), gl.STATIC_DRAW);

    gl.bindBuffer( gl.ARRAY_BUFFER, buffer3 );
    gl.bufferData( gl.ARRAY_BUFFER, directionArray, gl.STATIC_DRAW );

    gl.drawArrays(gl.POINTS, 0, numSprites);

    t2 = new Date()
    var fps = Math.floor(1000/(t2.valueOf()-t1.valueOf())+0.5);
    t1 = t2;
    console.log(fps);

    requestAnimationFrame(render);
}

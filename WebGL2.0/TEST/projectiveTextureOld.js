"use strict";

var canvas;
var gl;

var numVertices  = 36;

var texSize = 256;

var xAxis = 0;
var yAxis = 1;
var zAxis = 2;
var axis = 0;
var theta =[-30, -45, 0];

var thetaLoc;

var angle = 45;
var deltaAngle = 0.5;

var flag = false;
var lflag = false;

var program;

// Create a checkerboard pattern using floats


var image1 = new Array()
    for (var i =0; i<texSize; i++)  image1[i] = new Array();
    for (var i =0; i<texSize; i++)
        for ( var j = 0; j < texSize; j++)
           image1[i][j] = new Float32Array(4);
    for (var i =0; i<texSize; i++) for (var j=0; j<texSize; j++) {
        var c = (((i & 0x8) == 0) ^ ((j & 0x8)  == 0));
        image1[i][j] = [c, c, c, 1];
    }
    for(var i =0; i<texSize; i++) {
      image1[0][i] = image1[texSize-1][i] = image1[i][0] = image1[i][texSize-1] = [0, 0, 0, 1];
    }
    for (var i =0; i<texSize; i++)
        for ( var j = 0; j < texSize; j++) {
          if(Math.sqrt((i-texSize/2)*(i-texSize/2)+(j-texSize/2)*(j-texSize/2))>0.5*texSize)   image1[i][j] = [0, 0, 0, 1];
        }

// Convert floats to ubytes for texture

var image2 = new Uint8Array(4*texSize*texSize);

    for ( var i = 0; i < texSize; i++ )
        for ( var j = 0; j < texSize; j++ )
           for(var k =0; k<4; k++)
                image2[4*texSize*i+4*j+k] = 255*image1[i][j][k];

var pointsArray = [];
var texCoordsArray = [];
var normalsArray = [];

var texCoord = [
    vec2(0, 0),
    vec2(0, 1),
    vec2(1, 1),
    vec2(1, 0)
];


var vertices = [
    vec4( -0.5, -0.5,  0.5, 1.0 ),
    vec4( -0.5,  0.5,  0.5, 1.0 ),
    vec4( 0.5,  0.5,  0.5, 1.0 ),
    vec4( 0.5, -0.5,  0.5, 1.0 ),
    vec4( -0.5, -0.5, -0.5, 1.0 ),
    vec4( -0.5,  0.5, -0.5, 1.0 ),
    vec4( 0.5,  0.5, -0.5, 1.0 ),
    vec4( 0.5, -0.5, -0.5, 1.0 )
];

window.onload = init;

function configureTexture(image) {
    var texture = gl.createTexture();
    gl.activeTexture( gl.TEXTURE0 );
    gl.bindTexture( gl.TEXTURE_2D, texture );
    //gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, texSize, texSize, 0,
        gl.RGBA, gl.UNSIGNED_BYTE, image);
    gl.generateMipmap( gl.TEXTURE_2D );
    gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
       gl.NEAREST_MIPMAP_LINEAR );
    gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST );
    gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);

    gl.uniform1i(gl.getUniformLocation(program, "texture"), 0);
}

function quad(a, b, c, d) {

      var t1 = subtract(vertices[b], vertices[a]);
      var t2 = subtract(vertices[c], vertices[b]);
      var normal = cross(t1, t2);
      var normal = vec3(normal);

     pointsArray.push(vertices[a]);
     texCoordsArray.push(texCoord[0]);
     normalsArray.push(normal);

     pointsArray.push(vertices[b]);
     texCoordsArray.push(texCoord[1]);
     normalsArray.push(normal);

     pointsArray.push(vertices[c]);
     texCoordsArray.push(texCoord[2]);
     normalsArray.push(normal);

     pointsArray.push(vertices[a]);
     texCoordsArray.push(texCoord[0]);
     normalsArray.push(normal);


     pointsArray.push(vertices[c]);
     texCoordsArray.push(texCoord[2]);
     normalsArray.push(normal);

     pointsArray.push(vertices[d]);
     texCoordsArray.push(texCoord[3]);
     normalsArray.push(normal);
}


function colorCube()
{
    quad( 1, 0, 3, 2 );
    quad( 2, 3, 7, 6 );
    quad( 3, 0, 4, 7 );
    quad( 6, 5, 1, 2 );
    quad( 4, 5, 6, 7 );
    quad( 5, 4, 0, 1 );
}


function init() {
    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }

    gl.viewport( 0, 0, canvas.width, canvas.height );
    gl.clearColor( 0.5, 0.5, 0.5, 1.0 );

    gl.enable(gl.DEPTH_TEST);

    //
    //  Load shaders and initialize attribute buffers
    //
    program = initShaders( gl, "vertex-shader", "fragment-shader" );

    gl.useProgram( program );

    colorCube();

    var vBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer);
    gl.bufferData( gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation( program, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(positionLoc);

    var tBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, tBuffer);
    gl.bufferData( gl.ARRAY_BUFFER, flatten(texCoordsArray), gl.STATIC_DRAW );

    var texCoordLoc = gl.getAttribLocation( program, "aTexCoord");
    gl.vertexAttribPointer(texCoordLoc, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(texCoordLoc);

    var nBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, nBuffer );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW );

    var normalLoc = gl.getAttribLocation( program, "aNormal" );
    gl.vertexAttribPointer( normalLoc, 3, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( normalLoc );


    configureTexture(image2);

    thetaLoc = gl.getUniformLocation(program, "theta");

    document.getElementById("ButtonX").onclick = function(){axis = xAxis;};
    document.getElementById("ButtonY").onclick = function(){axis = yAxis;};
    document.getElementById("ButtonZ").onclick = function(){axis = zAxis;};
    document.getElementById("ButtonT").onclick = function(){flag = !flag;};
    document.getElementById("ButtonL").onclick = function(){lflag = !lflag;};

    render();
}

var render = function(){

    gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    if(lflag) angle+= deltaAngle;

    gl.uniform1f(gl.getUniformLocation(program, "angle"), angle);

    if(flag) theta[axis] += 0.5;

    var modelViewMatrix = mat4();

    modelViewMatrix = mult(modelViewMatrix, rotateX(theta[xAxis] ));
    modelViewMatrix = mult(modelViewMatrix, rotateY(theta[yAxis]));
    modelViewMatrix = mult(modelViewMatrix, rotateZ(theta[zAxis]));
    gl.uniformMatrix4fv( gl.getUniformLocation(program,
            "modelViewMatrix"), false, flatten(modelViewMatrix) );

    var projectionMatrix = mat4(); //default orthographic projection

    gl.uniformMatrix4fv( gl.getUniformLocation(program,
            "projectionMatrix"), false, flatten(projectionMatrix) );

    gl.drawArrays( gl.TRIANGLES, 0, numVertices );


    requestAnimationFrame(render);
}

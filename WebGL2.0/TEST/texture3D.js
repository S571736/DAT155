"use strict";

var canvas;
var gl;

var numVertices  = 36;

var texSize = 64;

// Create a checkerboard pattern using floats


var image3 = new Uint8Array(4*texSize*texSize*texSize);

for(var i=0; i<texSize; i++)
    for(var j=0; j<texSize; j++)
        for(var k=0; k<texSize; k++) {
             var x = (i-texSize/2)/texSize;
             var y = (j-texSize/2)/texSize;
             var z = (k-texSize/2)/texSize;
     //image3[4*(i+texSize*j+texSize*texSize*k)+n] = 256*Math.random();
     //image3[4*(i+texSize*j+texSize*texSize*k)] = Math.max(0,512*x) ;
     //image3[4*(i+texSize*j+texSize*texSize*k)+1] = Math.max(0, 512*y);
     //image3[4*(i+texSize*j+texSize*texSize*k)+2] = Math.max(0, 512*z);
     image3[4*(i+texSize*j+texSize*texSize*k)] = 256*Math.random() ;
     image3[4*(i+texSize*j+texSize*texSize*k)+1] = 256*Math.random();
     image3[4*(i+texSize*j+texSize*texSize*k)+2] = 256*Math.random();
     image3[4*(i+texSize*j+texSize*texSize*k)+3] = 255;
   }

var pointsArray = [];


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


var xAxis = 0;
var yAxis = 1;
var zAxis = 2;
var axis = xAxis;

var theta = vec3(45.0, 45.0, 45.0);

var thetaLoc;



function quad(a, b, c, d) {
     pointsArray.push(vertices[a]);
     pointsArray.push(vertices[b]);
     pointsArray.push(vertices[c]);
     pointsArray.push(vertices[a]);
     pointsArray.push(vertices[c]);
     pointsArray.push(vertices[d]);
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
    gl.clearColor( 1.0, 1.0, 1.0, 1.0 );

    gl.enable(gl.DEPTH_TEST);

    //
    //  Load shaders and initialize attribute buffers
    //
    var program = initShaders( gl, "vertex-shader", "fragment-shader" );
    gl.useProgram( program );

    colorCube();

    var vBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer);
    gl.bufferData( gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW);
    var positionLoc = gl.getAttribLocation( program, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(positionLoc);


    var texture3D = gl.createTexture();
    gl.activeTexture( gl.TEXTURE0 );
    gl.bindTexture( gl.TEXTURE_3D, texture3D );
    gl.texParameteri(gl.TEXTURE_3D, gl.TEXTURE_BASE_LEVEL, 0);
    gl.texParameteri(gl.TEXTURE_3D, gl.TEXTURE_MAX_LEVEL, Math.log2(texSize));
    gl.texImage3D(gl.TEXTURE_3D, 0, gl.RGBA, texSize, texSize, texSize, 0,
        gl.RGBA, gl.UNSIGNED_BYTE, image3);
    gl.texParameteri( gl.TEXTURE_3D, gl.TEXTURE_MIN_FILTER,
        gl.LINEAR_MIPMAP_LINEAR );
    gl.texParameteri( gl.TEXTURE_3D, gl.TEXTURE_MAG_FILTER, gl.LINEAR );
    gl.generateMipmap( gl.TEXTURE_3D );


    gl.uniform1i( gl.getUniformLocation(program, "textureMap3D"), 0);

    thetaLoc = gl.getUniformLocation(program, "theta");

    document.getElementById("ButtonX").onclick = function(){axis = xAxis;};
    document.getElementById("ButtonY").onclick = function(){axis = yAxis;};
    document.getElementById("ButtonZ").onclick = function(){axis = zAxis;};

    render();
}

var render = function() {
    gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    theta[axis] += 2.0;
    gl.uniform3fv(thetaLoc, theta);
    gl.drawArrays( gl.TRIANGLES, 0, numVertices );
    requestAnimationFrame(render);
}

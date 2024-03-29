
var gl;

var n = 64;

var pointsArray = [];

var color;

var near = -2;
var far = 2;

var time = 0.0;
var dt = 5.0;

const black = vec4(0.0, 0.0, 0.0, 1.0);
const red = vec4(1.0, 0.0, 0.0, 1.0);

const at = vec3(0, 0, 0);
const up = vec3(0.0, 1.0, 0.0);

var left = -1.5;
var right = 1.5;
var ytop = 1.5;
var bottom = -1.5;

var modeViewMatrix, projectionMatrix;
var modelViewMatrixLoc, projectionMatrixLoc;

var data = {};
for(var i =0; i< n; i++) data[i] = [];
for(var i=0; i<n; i++) for(var j=0; j<n; j++) data[i][j] = 0.0;

window.onload = function init()
{
    var canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }

    gl.viewport( 0, 0, canvas.width, canvas.height );

    gl.clearColor( 1.0, 1.0, 1.0, 1.0 );

    // enable depth testing and polygon offset
    // so lines will be in front of filled triangles

    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(1.0, 2.0);

// vertex array of nRows*nColumns quadrilaterals
// (two triangles/quad) from data


  for(var i=0; i<n-1; i++) {
      for(var j=0; j<n-1;j++) {
          pointsArray.push( vec4(2*i/n-1, 2*data[i][j]-1, 2*j/n-1, 1.0));
          pointsArray.push( vec4(2*(i+1)/n-1, 2*data[(i+1)][j]-1, 2*j/n-1, 1.0));
          pointsArray.push( vec4(2*(i+1)/n-1, 2*data[(i+1)][j+1]-1, 2*(j+1)/n-1, 1.0));
          pointsArray.push( vec4(2*i/n-1, 2*data[i][j+1]-1, 2*(j+1)/n-1, 1.0) );
      }
    }

    //
    //  Load shaders and initialize attribute buffers
    //
    var program = initShaders( gl, "vertex-shader", "fragment-shader" );
    gl.useProgram( program );


    var vBufferId = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, vBufferId );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation( program, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( positionLoc );

    color = gl.getUniformLocation(program, "color");

    modelViewMatrixLoc = gl.getUniformLocation( program, "modelViewMatrix" );
    projectionMatrixLoc = gl.getUniformLocation( program, "projectionMatrix" );
    timeLoc = gl.getUniformLocation(program, "time");

    render();

}


function render()
{
    gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    var eye = vec3(1, 1, 1);
    time += dt;
    gl.uniform1f(timeLoc, time);

    modelViewMatrix = lookAt( eye, at, up );
    projectionMatrix = ortho( left, right, bottom, ytop, near, far );

    gl.uniformMatrix4fv( modelViewMatrixLoc, false, flatten(modelViewMatrix) );
    gl.uniformMatrix4fv( projectionMatrixLoc, false, flatten(projectionMatrix) );

    // draw each quad as two filled red triangles
    // and then as two black line loops

    for(var i=0; i<pointsArray.length; i+=4) {
        gl.uniform4fv(color, flatten(red));
        gl.drawArrays( gl.TRIANGLE_FAN, i, 4 );
        gl.uniform4fv(color, flatten(black));
        gl.drawArrays( gl.LINE_LOOP, i, 4 );
    }


    requestAnimationFrame(render);
}

"use strict";

var gl;

var nRows = 50;
var nColumns = 50;

var time = 0.0;
var dt = 0.4;

var timeLoc;

// data for radial hat function: sin(Pi*r)/(Pi*r)

var data = [];
var normalx = [];
var normaly = [];

for( var i = 0; i < nRows; ++i ) {
    data.push( [] );
    var x = 2.0*i/nRows-1.0;

    for( var j = 0; j < nColumns; ++j ) {
        var y = 2.0*j/nRows-1.0;

        data[i][j] =  0.2*Math.sin(10.0*x)*Math.cos(10.0*y);
         //data[i][j] += 0.7;
    }
}

for( var i = 0; i < nRows-1; ++i ) {
    normalx.push( [] );
    normaly.push( [] );
    var x = 2.0*i/nRows-1.0;

    for( var j = 0; j < nColumns-1; ++j ) {
        var y = 2.0*j/nRows-1.0;

        normalx[i][j] = 50.0*(data[i+1][j]-data[i][j]);
        normaly[i][j] =  50.0*(data[i][j+1]-data[i][j]);

    }
}

var pointsArray = [];
var normalsArray = [];

var color;

const black = vec4(0.0, 0.0, 0.0, 1.0);
const cyan = vec4(0.0, 1.0, 1.0, 1.0);

const at = vec3(0.0, 0.0, 0.0);
const up = vec3(0.0, 1.0, 0.0);
const eye = vec3(3.0, 3.0, 10.0);

var left = -1.0;
var right = 1.0;
var ytop = 1.0;
var bottom = -1.0;
var near = -20;
var far = 20;

var modeViewMatrix, projectionMatrix;
var modelViewMatrixLoc, projectionMatrixLoc;




window.onload = function init()
{
    var canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }

    gl.viewport( 0, 0, canvas.width, canvas.height );

    gl.clearColor( 0.5, 0.5, 0.5, 1.0 );

    // enable depth testing and polygon offset
    // so lines will be in front of filled triangles

    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(1.0, 2.0);

// vertex array of nRows*nColumns quadrilaterals
// (two triangles/quad) from data

    for(var i=0; i<nRows-2; i++) {
        for(var j=0; j<nColumns-2;j++) {
            pointsArray.push( vec4(2*i/nRows-1, data[i][j], 2*j/nColumns-1, 1.0));
            pointsArray.push( vec4(2*(i+1)/nRows-1, data[i+1][j], 2*j/nColumns-1, 1.0));
            pointsArray.push( vec4(2*(i+1)/nRows-1, data[i+1][j+1], 2*(j+1)/nColumns-1, 1.0));
            pointsArray.push( vec4(2*i/nRows-1, data[i][j+1], 2*(j+1)/nColumns-1, 1.0) );
            normalsArray.push( vec4(normalx[i][j], normaly[i][j], 1.0, 0.0));
            normalsArray.push( vec4(normalx[i+1][j], normaly[i+1][j], 1.0, 0.0));
            normalsArray.push( vec4(normalx[i+1][j+1], normaly[i+1][j+1], 1.0, 0.0));
            normalsArray.push( vec4(normalx[i][j+1], normaly[i][j+1], 1.0, 0.0));
    }
}
    //
    //  Load shaders and initialize attribute buffers
    //
    var program = initShaders( gl, "vertex-shader", "fragment-shader" );
    gl.useProgram( program );

    var nBufferId = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, nBufferId );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW);

    var normalLoc = gl.getAttribLocation( program, "aNormal" );
    gl.vertexAttribPointer( normalLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( normalLoc );


    var vBufferId = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, vBufferId );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation( program, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( positionLoc );


    color = gl.getUniformLocation(program, "color");

    modelViewMatrixLoc = gl.getUniformLocation( program, "modelViewMatrix" );
    projectionMatrixLoc = gl.getUniformLocation( program, "projectionMatrix" );

    timeLoc = gl.getUniformLocation( program, "time" );

    render();

}


function render()
{
    gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
/*
    var eye = vec3( radius*Math.sin(theta)*Math.cos(phi),
                    radius*Math.sin(theta)*Math.sin(phi),
                    radius*Math.cos(theta));
*/

    var modelViewMatrix = lookAt( eye, at, up );
    var projectionMatrix = ortho( left, right, bottom, ytop, near, far );

    gl.uniformMatrix4fv( modelViewMatrixLoc, false, flatten(modelViewMatrix) );
    gl.uniformMatrix4fv( projectionMatrixLoc, false, flatten(projectionMatrix) );

    // draw each quad as two filled red triangles
    // and then as two black line loops

    for(var i=0; i<pointsArray.length; i+=4) {
        gl.uniform4fv(color, flatten(cyan));
        gl.drawArrays( gl.TRIANGLE_FAN, i, 4 );
        gl.uniform4fv(color, flatten(black));
        gl.drawArrays( gl.LINE_LOOP, i, 4 );
    }
    time += dt;
    gl.uniform1f(timeLoc, time);
    requestAnimationFrame(render);
}

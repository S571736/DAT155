"use strict";

var canvas;
var gl;
var points;
var colors;

var NumPoints = 5000;

window.onload = function init()
{
    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }


    //
    //  Initialize our data for the Sierpinski Gasket
    //

    // First, initialize the vertices of our 3D gasket

    var vertices = [
        vec3( -0.5, -0.5, -0.5 ),
        vec3(  0.5, -0.5, -0.5 ),
        vec3(  0.0,  0.5,  0.0 ),
        vec3(  0.0, -0.5, 0.5 ),
    ];


    points = [ vec3( 0.0, 0.0, 0.0 ) ];
    colors = [vec4(0.5, 0.5, 0.5, 1.0)];

    for ( var i = 0; points.length < NumPoints; ++i ) {
        var j = Math.floor(Math.random() * 4);

        var newPoint = mix(points[i], vertices[j], 0.5);
        points.push( newPoint);
        colors.push(vec4((1.0+newPoint[0])/2.0, (1.0+newPoint[1])/2.0, (1.0+newPoint[2])/2.0, 1.0));
    }

    //
    //  Configure WebGL
    //
    gl.viewport( 0, 0, canvas.width, canvas.height );
    gl.clearColor( 1.0, 1.0, 1.0, 1.0 );
    gl.enable( gl.DEPTH_TEST );

    //  Load shaders and initialize attribute buffers

    var program = initShaders( gl, "vertex-shader", "fragment-shader" );
    gl.useProgram( program );

    // Load the data into the GPU

    var bufferId = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, bufferId );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(points), gl.STATIC_DRAW );

    // Associate out shader variables with our data buffer

    var positionLoc = gl.getAttribLocation( program, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 3, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( positionLoc );

    var cBufferId = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, cBufferId );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(colors), gl.STATIC_DRAW );

    // Associate out shader variables with our data buffer

    var colorLoc = gl.getAttribLocation( program, "aColor" );
    gl.vertexAttribPointer( colorLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( colorLoc );

    render();
};


function render()
{
    gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT );
    gl.drawArrays( gl.POINTS, 0, points.length );
}

"use strict";

var canvas;
var gl;

var points = [];
var colors = [];
var newColors = [];

var NumTimesToSubdivide = 3;

window.onload = function init()
{
    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }

      var bufferData = function(shaderName, data, components) {
        var buffer = gl.createBuffer();
        gl.bindBuffer( gl.ARRAY_BUFFER, buffer );
        gl.bufferData( gl.ARRAY_BUFFER, flatten(data), gl.STATIC_DRAW );

        var bufferLoc = gl.getAttribLocation( program, shaderName );
        gl.vertexAttribPointer( bufferLoc, components, gl.FLOAT, false, 0, 0 );
        gl.enableVertexAttribArray( bufferLoc );
      return buffer;
    }

      var newBufferData = function(buffer, data) {
        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.bufferData(gl.ARRAY_BUFFER, flatten(data), gl.STATIC_DRAW);
      }


    //
    //  Initialize our data for the Sierpinski Gasket
    //

    // First, initialize the vertices of our 3D gasket
    // Four vertices on unit circle
    // Intial tetrahedron with equal length sides

    var vertices = [
        vec3(  0.0000,  0.0000, -1.0000 ),
        vec3(  0.0000,  0.9428,  0.3333 ),
        vec3( -0.8165, -0.4714,  0.3333 ),
        vec3(  0.8165, -0.4714,  0.3333 )
    ];

    divideTetra( vertices[0], vertices[1], vertices[2], vertices[3],
                 NumTimesToSubdivide);

    //
    //  Configure WebGL
    //
    gl.viewport( 0, 0, canvas.width, canvas.height );
    gl.clearColor( 1.0, 1.0, 1.0, 1.0 );

    // enable hidden-surface removal

    gl.enable(gl.DEPTH_TEST);

    //  Load shaders and initialize attribute buffers

    var program = initShaders( gl, "vertex-shader", "fragment-shader" );
    gl.useProgram( program );

    // Create a buffer object, initialize it, and associate it with the
    //  associated attribute variable in our vertex shader

    var positionBuffer = bufferData("aPosition", points, 3);
    var colorBuffer = bufferData("aColor", colors, 3);

    //newBufferData(colorBuffer, newColors);

    render();
};

function triangle( a, b, c, color )
{

    // add colors and vertices for one triangle

    var baseColors = [
        vec3(1.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        vec3(0.0, 0.0, 1.0),
        vec3(0.0, 0.0, 0.0)
    ];

    colors.push( baseColors[color] );
    newColors.push( baseColors[(color+1)%3 ]);
    points.push( a );
    newColors.push( baseColors[(color+1)%3 ]);
    colors.push( baseColors[color] );
    newColors.push( baseColors[(color+1)%3 ]);
    points.push( b );
    colors.push( baseColors[color] );
    points.push( c );
}

function tetra( a, b, c, d )
{
    // tetrahedron with each side using
    // a different color

    triangle( a, c, b, 0 );
    triangle( a, c, d, 1 );
    triangle( a, b, d, 2 );
    triangle( b, c, d, 3 );
}

function divideTetra( a, b, c, d, count )
{
    // check for end of recursion

    if ( count === 0 ) {
        tetra( a, b, c, d );
    }

    // find midpoints of sides
    // divide four smaller tetrahedra

    else {
        var ab = mix( a, b, 0.5 );
        var ac = mix( a, c, 0.5 );
        var ad = mix( a, d, 0.5 );
        var bc = mix( b, c, 0.5 );
        var bd = mix( b, d, 0.5 );
        var cd = mix( c, d, 0.5 );

        --count;

        divideTetra(  a, ab, ac, ad, count );
        divideTetra( ab,  b, bc, bd, count );
        divideTetra( ac, bc,  c, cd, count );
        divideTetra( ad, bd, cd,  d, count );
    }
}


function render()
{
    gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.drawArrays( gl.TRIANGLES, 0, points.length );
    requestAnimationFrame(render);
}

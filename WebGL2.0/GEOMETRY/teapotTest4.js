"use strict";

var modelViewMatrix = [];
var projectionMatrix = [];

var axis = 0;
var xAxis = 0;
var yAxis = 1;
var zAxis = 2;
var theta = vec3(0, 0, 0);

var flag = true;

var program;
var canvas, render, gl;

var points = [];
var colors = [];
var texCoord = [];
var myTeapots = [];
var numTeapots = 20;

onload = function init()  {

    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }

    gl.viewport( 0, 0, canvas.width, canvas.height );

    gl.clearColor( 1.0, 1.0, 1.0, 1.0 );

    gl.enable(gl.DEPTH_TEST);

    for(var i =0; i<numTeapots; i++){
      myTeapots[i] = teapot(3);
      var s = 0.5*Math.random();
      myTeapots[i].scale(s, s, s);
      myTeapots[i].translate(3.0*(2.0*Math.random()-1.0), 3.0*(2.0*Math.random()-1.0), 3.0*(2.0*Math.random()-1.0));
      myTeapots[i].rotate(360*Math.random(), [2.0*Math.random() - 1.0, 2.0*Math.random() - 1.0, 2.0*Math.random() - 1.0]);
    }

    var texture = checkerboardTexture();
    points = myTeapots[0].TriangleVertices;
    colors = myTeapots[0].VertexColors;
    texCoord = myTeapots[0].TextureCoordinates;
    for(var i =1; i<numTeapots; i++) {
      points = points.concat(myTeapots[i].TriangleVertices);
      colors = colors.concat(myTeapots[i].VertexColors);
      texCoord = texCoord.concat(myTeapots[i].TextureCoordinates);
    }

    document.getElementById("ButtonX").onclick = function(){axis = xAxis;};
    document.getElementById("ButtonY").onclick = function(){axis = yAxis;};
    document.getElementById("ButtonZ").onclick = function(){axis = zAxis;};
    document.getElementById("ButtonT").onclick = function(){flag = !flag;};

    program = initShaders( gl, "vertex-shader", "fragment-shader" );
    gl.useProgram( program );

    var vBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer);
    gl.bufferData( gl.ARRAY_BUFFER, flatten(points), gl.STATIC_DRAW );


    var positionLoc = gl.getAttribLocation( program, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( positionLoc );


    var cBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, cBuffer);
    gl.bufferData( gl.ARRAY_BUFFER, flatten(colors), gl.STATIC_DRAW );

    var colorLoc = gl.getAttribLocation( program, "aColor" );
    gl.vertexAttribPointer( colorLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( colorLoc);

    var tBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, tBuffer);
    gl.bufferData( gl.ARRAY_BUFFER, flatten(texCoord), gl.STATIC_DRAW );

    var texCoordLoc = gl.getAttribLocation( program, "aTexCoord");
    gl.vertexAttribPointer(texCoordLoc, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(texCoordLoc);

    projectionMatrix = ortho(-4, 4, -4, 4, -200, 200);
    gl.uniformMatrix4fv( gl.getUniformLocation(program, "projectionMatrix"), false, flatten(projectionMatrix));

    render();
}

var render = function() {

      gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

      if(flag) theta[axis] += 0.5;

      modelViewMatrix = mat4();

      modelViewMatrix = mult(modelViewMatrix, rotate(theta[xAxis], vec3(1, 0, 0)));
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[yAxis], vec3(0, 1, 0)));
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[zAxis], vec3(0, 0, 1)));

      gl.uniformMatrix4fv( gl.getUniformLocation(program, "modelViewMatrix"), false,
          flatten(modelViewMatrix) );

      gl.drawArrays( gl.TRIANGLES, 0, points.length);
      requestAnimationFrame(render);
  }

"use strict";

var modelViewMatrix = [];
var projectionMatrix = [];

var nMatrix, nMatrixLoc;

var axis =0;

var axis = 0;
var xAxis = 0;
var yAxis = 1;
var zAxis = 2;
var theta = vec3(0, 0, 0);
var dTheta = 5.0;

var flag = true;

var program;
var canvas, render, gl;

var points = [];
var normals = [];

onload = function init()  {

    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }

    gl.viewport( 0, 0, canvas.width, canvas.height );

    gl.clearColor( 1.0, 1.0, 1.0, 1.0 );

    gl.enable(gl.DEPTH_TEST);

    var myTeapot = teapot(3);
    var myTeapot2 = teapot(3);
    myTeapot.scale(0.5, 0.5, 0.5);
    myTeapot2.scale(0.5, 0.5, 0.5);
    myTeapot.translate(-1.5, -0.0, -0.0);
    myTeapot2.translate(1.5, 0.0, 0.0);
    myTeapot.rotate(45, [1, 1, 1]);

    console.log(myTeapot.TriangleVertices.length);

    points = myTeapot.TriangleVertices;
    normals = myTeapot.Normals;
    points = points.concat(myTeapot2.TriangleVertices);
    normals = normals.concat(myTeapot2.Normals);

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


    var nBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, nBuffer);
    gl.bufferData( gl.ARRAY_BUFFER, flatten(normals), gl.STATIC_DRAW );

    var normalLoc = gl.getAttribLocation( program, "aNormal" );
    gl.vertexAttribPointer( normalLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( normalLoc);

    projectionMatrix = ortho(-4, 4, -4, 4, -200, 200);
    gl.uniformMatrix4fv( gl.getUniformLocation(program, "projectionMatrix"), false, flatten(projectionMatrix));
    nMatrixLoc = gl.getUniformLocation( program, "normalMatrix" );


    var myMaterial = goldMaterial();
    var myLight = light0();

    var ambientProduct = mult(myLight.lightAmbient, myMaterial.materialAmbient);
    var diffuseProduct = mult(myLight.lightDiffuse, myMaterial.materialDiffuse);
    var specularProduct = mult(myLight.lightSpecular, myMaterial.materialSpecular);


    gl.uniform4fv( gl.getUniformLocation(program, "ambientProduct"),ambientProduct );
    gl.uniform4fv( gl.getUniformLocation(program, "diffuseProduct"), diffuseProduct );
    gl.uniform4fv( gl.getUniformLocation(program, "specularProduct"), specularProduct);
    gl.uniform4fv( gl.getUniformLocation(program, "lightPosition"), myLight.lightPosition );
    gl.uniform1f( gl.getUniformLocation(program, "shininess"),myMaterial.materialShininess );


    render();
}

var render = function(){
      gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

      if(flag) theta[axis] += 0.5;

      modelViewMatrix = mat4();

      modelViewMatrix = mult(modelViewMatrix, rotate(theta[xAxis], vec3(1, 0, 0)));
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[yAxis], vec3(0, 1, 0)));
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[zAxis], vec3(0, 0, 1)));

      gl.uniformMatrix4fv( gl.getUniformLocation(program, "modelViewMatrix"), false,
          flatten(modelViewMatrix ));
      nMatrix = normalMatrix(modelViewMatrix);

      gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix) );

      gl.drawArrays( gl.TRIANGLES, 0, points.length);
      requestAnimationFrame(render);
  }

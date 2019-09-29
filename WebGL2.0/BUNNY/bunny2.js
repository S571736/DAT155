"use strict";

var shadedBunny = function() {

var canvas;
var gl;

var pointsArray = [];
var normalsArray = [];

for(var i=0; i<indices.length; i++) {
  var a = vec3(vertices[indices[i][0]][0], vertices[indices[i][0]][1], vertices[indices[i][0]][2]);
  var b = vec3(vertices[indices[i][1]][0], vertices[indices[i][1]][1], vertices[indices[i][1]][2]);
  var c = vec3(vertices[indices[i][2]][0], vertices[indices[i][2]][1], vertices[indices[i][2]][2]);
  var t1 = subtract(a, b);
  var t2 = subtract(c, b);
  var normal = cross(t1, t2);
  normal = vec3(normal);
  for(var j=0; j<3; j++) {
    pointsArray.push(vertices[indices[i][j]]);
    normalsArray.push(normal);
  };
};

var lightPosition = vec4(1.0, 1.0, -3.0, 0.0 );
var lightAmbient = vec4(0.2, 0.2, 0.2, 1.0 );
var lightDiffuse = vec4( 1.0, 1.0, 1.0, 1.0 );
var lightSpecular = vec4( 1.0, 1.0, 1.0, 1.0 );

var materialAmbient = vec4( 1.0, 0.0, 1.0, 1.0 );
var materialDiffuse = vec4( 1.0, 0.8, 0.0, 1.0);
var materialSpecular = vec4( 1.0, 0.8, 0.0, 1.0 );
var materialShininess = 100.0;

var ctm;
var ambientColor, diffuseColor, specularColor;
var modelViewMatrix, projectionMatrix;
var viewerPos;
var program;

var xAxis = 0;
var yAxis = 1;
var zAxis = 2;
var axis = 0;
var theta = vec3(0, 0, 0);

var thetaLoc;

var flag = false;

window.onload = function init() {
    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }

    gl.viewport( 0, 0, canvas.width, canvas.height );
    gl.clearColor( 1.0, 1.0, 1.0, 1.0 );

    gl.enable(gl.DEPTH_TEST);

    //
    //  Load shaders and initialize attribute buffers
    //
    program = initShaders( gl, "vertex-shader", "fragment-shader" );
    gl.useProgram( program );

    var nBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, nBuffer );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW );

    var normalLoc = gl.getAttribLocation( program, "aNormal" );
    gl.vertexAttribPointer( normalLoc, 3, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( normalLoc );

    var vBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW );

    var positionLoc = gl.getAttribLocation(program, "aPosition");
    gl.vertexAttribPointer(positionLoc, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(positionLoc);

    thetaLoc = gl.getUniformLocation(program, "theta");

    viewerPos = vec3(0.0, 0.0, -20.0 );

    projectionMatrix = ortho(-1, 1, -1, 1, -100, 100);

    var ambientProduct = mult(lightAmbient, materialAmbient);
    var diffuseProduct = mult(lightDiffuse, materialDiffuse);
    var specularProduct = mult(lightSpecular, materialSpecular);

    document.getElementById("ButtonX").onclick = function(){axis = xAxis;};
    document.getElementById("ButtonY").onclick = function(){axis = yAxis;};
    document.getElementById("ButtonZ").onclick = function(){axis = zAxis;};
    document.getElementById("ButtonT").onclick = function(){flag = !flag;};

    gl.uniform4fv(gl.getUniformLocation(program, "ambientProduct"),
       ambientProduct);
    gl.uniform4fv(gl.getUniformLocation(program, "diffuseProduct"),
       diffuseProduct );
    gl.uniform4fv(gl.getUniformLocation(program, "specularProduct"),
       specularProduct );
    gl.uniform4fv(gl.getUniformLocation(program, "lightPosition"),
       lightPosition );

    gl.uniform1f(gl.getUniformLocation(program,
       "shininess"),materialShininess);

    gl.uniformMatrix4fv( gl.getUniformLocation(program, "projectionMatrix"),
       false, flatten(projectionMatrix));
    render();
}

var render = function(){

    gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    if(flag) theta[axis] += 2.0;

    modelViewMatrix = mat4();
    modelViewMatrix = mult(modelViewMatrix, rotate(theta[xAxis], vec3(1, 0, 0) ));
    modelViewMatrix = mult(modelViewMatrix, rotate(theta[yAxis], vec3(0, 1, 0) ));
    modelViewMatrix = mult(modelViewMatrix, rotate(theta[zAxis], vec3(0, 0, 1) ));

    //console.log(modelView);

    gl.uniformMatrix4fv( gl.getUniformLocation(program,
            "modelViewMatrix"), false, flatten(modelViewMatrix) );

    gl.drawArrays( gl.TRIANGLES, 0, pointsArray.length );


    requestAnimationFrame(render);
}

}

shadedBunny();

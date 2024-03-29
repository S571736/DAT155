
var gl;

var pointsArray = [];
var normalsArray = [];

var vBuffer, nBuffer;
var vPosition, vNormal;

var color;

var near = -2;
var far = 2;
var radius = 1.0;
var theta  = -0.33;
var phi    = -1.4;
var dr = 5.0 * Math.PI/180.0;

const black = vec4(0.0, 0.0, 0.0, 1.0);
const red = vec4(1.0, 0.0, 0.0, 1.0);

var at = vec3(0, 0, 0);
var up = vec3(0.0, 1.0, 0.0);

var left = -1.2;
var right = 1.2;
var ytop = 1.0;
var bottom = -1.0;

var scale = 7.0;

var lightPosition = vec4(1.0, 1.0, 1.0, 0.0 );
var lightAmbient = vec4(0.2, 0.2, 0.2, 1.0 );
var lightDiffuse = vec4( 1.0, 1.0, 1.0, 1.0 );
var lightSpecular = vec4( 1.0, 1.0, 1.0, 1.0 );

var materialAmbient = vec4( 1.0, 0.0, 1.0, 1.0 );
var materialDiffuse = vec4( 1.0, 0.8, 0.0, 1.0 );
var materialSpecular = vec4( 1.0, 1.0, 1.0, 1.0 );
var materialShininess = 100.0;

var ambientProduct = mult(lightAmbient, materialAmbient);
var diffuseProduct = mult(lightDiffuse, materialDiffuse);
var specularProduct = mult(lightSpecular, materialSpecular);

var modeViewMatrix, projectionMatrix;
var modelViewMatrixLoc, projectionMatrixLoc;
var nMatrix, nMatrixLoc;

var data = data256;

loadPoints = function(n) {
  for(var i=0; i<n-1; i++) {
      for(var j=0; j<n-1;j++) {
        var a = vec4(2*i/n-1, 2*data[i*n+j]-1, 2*j/n-1, 1.0);
        var b = vec4(2*(i+1)/n-1, 2*data[(i+1)*n+j]-1, 2*j/n-1, 1.0);
        var c = vec4(2*(i+1)/n-1, 2*data[(i+1)*n+j+1]-1, 2*(j+1)/n-1, 1.0);
        var d = vec4(2*i/n-1, 2*data[i*n+j+1]-1, 2*(j+1)/n-1, 1.0);

        a[1] /= scale;
        b[1] /= scale;
        c[1] /= scale;
        d[1] /= scale;

          pointsArray.push(a);
          pointsArray.push(b);
          pointsArray.push(c);

          var t1 = subtract(b, a);
          var t2 = subtract(c, a);
          var normal = normalize(cross(t2, t1));
          normal = vec4(normal[0], normal[1], normal[2], 0.0);

          normalsArray.push(normal);
          normalsArray.push(normal);
          normalsArray.push(normal);

          pointsArray.push(c);
          pointsArray.push(d);
          pointsArray.push(a);

          t1 = subtract(d, c);
          t2 = subtract(a, c);
          normal = normalize(cross(t2, t1));
          normal = vec4(normal[0], normal[1], normal[2], 0.0);

          normalsArray.push(normal);
          normalsArray.push(normal);
          normalsArray.push(normal);
      }
    }
}

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

// vertex array of nRows*nColumns quadrilaterals
// (two triangles/quad) from data

    loadPoints(256);
    //
    //  Load shaders and initialize attribute buffers
    //
    var program = initShaders( gl, "vertex-shader", "fragment-shader" );
    gl.useProgram( program );

    nBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, nBuffer );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW);

    normalLoc = gl.getAttribLocation( program, "aNormal" );
    gl.vertexAttribPointer( normalLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( normalLoc );

    vBuffer = gl.createBuffer();
    gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer );
    gl.bufferData( gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW);

    positionLoc = gl.getAttribLocation( program, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( positionLoc );

    color = gl.getUniformLocation(program, "color");

    modelViewMatrixLoc = gl.getUniformLocation( program, "modelViewMatrix" );
    projectionMatrixLoc = gl.getUniformLocation( program, "projectionMatrix" );
    nMatrixLoc = gl.getUniformLocation( program, "normalMatrix" );

// buttons for moving viewer and changing size

    document.getElementById("Button1").onclick = function(){theta += dr;};
    document.getElementById("Button2").onclick = function(){theta -= dr;};
    document.getElementById("Button3").onclick = function(){phi += dr;};
    document.getElementById("Button4").onclick = function(){phi -= dr;};

    document.getElementById("Res64").onclick = function() {
        data = data64;
        pointsArray = [];
        normalsArray = [];
        loadPoints(64);
        gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer );
        gl.bufferSubData(gl.ARRAY_BUFFER, 0, flatten(pointsArray));
        gl.bindBuffer( gl.ARRAY_BUFFER, nBuffer );
        gl.bufferSubData(gl.ARRAY_BUFFER, 0, flatten(normalsArray));
      };


      document.getElementById("Res128").onclick = function() {
          data = data128;
          pointsArray = [];
          normalsArray = [];
          loadPoints(128);
          gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer );
          gl.bufferSubData(gl.ARRAY_BUFFER, 0, flatten(pointsArray));
          gl.bindBuffer( gl.ARRAY_BUFFER, nBuffer );
          gl.bufferSubData(gl.ARRAY_BUFFER, 0, flatten(normalsArray));
        };


        document.getElementById("Res256").onclick = function() {
            data = data256;
            pointsArray = [];
            normalsArray = [];
            loadPoints(256);
            gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer );
            gl.bufferSubData(gl.ARRAY_BUFFER, 0, flatten(pointsArray));
            gl.bindBuffer( gl.ARRAY_BUFFER, nBuffer );
            gl.bufferSubData(gl.ARRAY_BUFFER, 0, flatten(normalsArray));
          };

          gl.uniform4fv( gl.getUniformLocation(program,
             "ambientProduct"), ambientProduct );
          gl.uniform4fv( gl.getUniformLocation(program,
             "diffuseProduct"), diffuseProduct );
          gl.uniform4fv( gl.getUniformLocation(program,
             "specularProduct"), specularProduct );
          gl.uniform4fv( gl.getUniformLocation(program,
             "lightPosition"), lightPosition );
          gl.uniform1f( gl.getUniformLocation(program,
             "shininess"),materialShininess );

    render();

}


function render()
{
    gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    var eye = vec3( radius*Math.sin(theta)*Math.cos(phi),
                    radius*Math.sin(theta)*Math.sin(phi),
                    radius*Math.cos(theta));


    modelViewMatrix = lookAt( eye, at, up );
    projectionMatrix = ortho( left, right, bottom, ytop, near, far );
    nMatrix = normalMatrix(modelViewMatrix);

    gl.uniformMatrix4fv( modelViewMatrixLoc, false, flatten(modelViewMatrix) );
    gl.uniformMatrix4fv( projectionMatrixLoc, false, flatten(projectionMatrix) );
    gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix ));

    gl.drawArrays( gl.TRIANGLES, 0, pointsArray.length );
    requestAnimationFrame(render);
}

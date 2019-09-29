// 2D Test
    var m2 = mat2(1,2, 3, 4);
    console.log(m2);
    console.log ("m2[0][0] ",m2[0][0]);
    console.log ("m2[0][1] ",m2[0][1]);
    console.log ("m2[1][0] ",m2[1][0]);
    console.log ("m2[1][1] ",m2[1][1]);
    console.log(m2[0]);
    console.log(m2[1]);
    console.log(m2[0][0], m2[0][1]);
    console.log(m2[1][0], m2[1][1]);

    var c2 = mat2(m2);
    c2[0][0] = 5.0;
    console.log(m2[0][0]);
    console.log(m2.length);
    console.log(c2.data);
    console.log("2D Matrix");
    printm(m2);
    console.log("determinant = ",det(m2));
    var a2 = mat2();
    a2 = inverse(m2);
    console.log("inverse")
    printm(a2);
    var b2 = mat2();
    console.log("matrix * matrix inverse");
    b2 = mult(a2, m2);
    printm(b2);

    var v2 = vec2(1, 2);
    console.log("2D vector", v2);
    var mv2 = mult(m2, v2);
    console.log("matrix vector mult", mv2);
    console.log("add test ");
     printm(add(m2, m2))

// 3D Test


    var m3 = mat3(0,2,3,4,5,6,7,8,9);
    var v3 = vec3(1, 2, 3);
    console.log("3D Matrix");
    printm(m3);
    console.log(v3);
    console.log("3D mat vec mult");
    console.log(mult(m3, v3));
    console.log("determinant = ",det(m3));
    var a3 = mat3();
    a3 = inverse(m3);
    console.log("inverse")
    printm(a3);
    var b3 = mat3();
    console.log("matrix * matrix inverse");
    b3 = mult(a3, m3);
    printm(b3);

    //4D Test
    m4 = mat4(0, 1, 2, 0, 3, 4, 5, 0, 6, 7, 8, 0, 0, 0, 0, 1);

    console.log("4D matrix");

    printm(m4);
        var axis = [1.0, 1.0, 1.0];
    var v4 = vec4(1, 2, 3, 4);
    console.log("vector");
    console.log(v4[0], v4[1], v4[2], v4[3]);
    console.log("mat4 vec4 mult");
    printm(m4);
    console.log(v4);
    console.log(mult(m4, v4));
    printm(translate(1.0, 2.0, 3.0));
    printm(rotate(30.0, axis));
    m4 = mult(translate(1.0, 2.0, 3.0),rotate(30.0, axis));
    console.log("4D Matrix");
    printm(m4);
    console.log("determinant = ",det(m4));
    var a4 = mat4();
    a4 = inverse(m4);
    console.log("inverse")
    printm(a4);
    var b4 = mat4();
    console.log("matrix * matrix inverse");
    b4 = mult(a4, m4);
    printm(b4);
    console.log("normal matrix transpose(inverse)");
    printm(transpose(inverse(m4)));
    console.log("normal matrix inverse(transpose)");
    printm(inverse(transpose(m4)));
    console.log("4 x 4 normal matrix");
    var n = mat4();
    n = normalMatrix(m4);
    printm(n);
    console.log("3 x 3 normal matrix");
    var n3 = mat4();
    n3 = normalMatrix(m4, true);
    printm(n3);
    console.log("translation matrix");
    printm(translate(1, 2, 3));

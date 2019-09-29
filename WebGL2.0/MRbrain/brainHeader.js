"use strict";

var nRows = 256;
var nColumns = 256;
var nSlices = 108;

var data = new Uint8Array(nSlices*nRows*nColumns);

var max = 0;


var rawData = new Uint32Array([

]);

for (var k =0; k< nSlices; k++) for(var i=0; i<nRows;i++) for(var j=0; j<nColumns; j++) {
    if( rawData[k*nRows*nColumns+i*nColumns+j]>max) max = rawData[i*nColumns+j];
    //console.log(rawData[i*nColumns+j]);
    }
//console.log(max);

for (var k =0; k< nSlices; k++) for(var i=0; i<nRows;i++) for(var j=0; j<nColumns; j++) {
  data[k*nRows*nColumns+i*nColumns+j] = 255*(rawData[i*nColumns+j]/max);
//  console.log(data[i*nColumns+j]);
  }

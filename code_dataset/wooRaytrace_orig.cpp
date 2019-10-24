#include <string.h>
#include "mex.h"
#include <math.h>
#include <algorithm>
#include <iostream>

using namespace std;

#define MAX(a,b) ((a>=b)?a:b) 
#define MIN(a,b) ((a<=b)?a:b)

int sub2ind(const int X, const int Y, const int Z, const double * gridSize); // convert subscript to linear index - Y-dimension taken as first dimension
int boxIntersectTest(double const *gridSize, double const *gridBounds, double const *lineCoord, double & tmin, double & tmax); // box-line intersect test

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
// Trace a line segment defined by two points through a voxel grid (using Woo's raytracing algorithm).
// Determine linear indexes of voxels intersected by the line segment.

// Inputs
//First input: dimensions of the voxel grid [Nx Ny Nz]
//Second input: a 1-by-6 matrix with voxel grid boundaries [xMin yMin zMin xMax yMax zMax] (must satisfy xMax>xMin, yMax>yMin, zMax>zMin)
//Third input: a 1-by-6 matrix with line segment coordinates [x1,y1,z1,x2,y2,z2]

//Output
//First output: a N-by-1 matrix containing linear indeces of the intersected voxels

// Version 1.0
// April 12, 2016
// Author: Ivan Klyuzhin

//Check the number of inputs and outputs
if (nlhs != 1) {
	mexErrMsgTxt("Error: Number of output arguments must be equal 1.");
}
if (nrhs != 3) {
	mexErrMsgTxt("Error: Number of input arguments must be equal 3.");
}
//Check sizes of the inputs
if ( (mxGetN(prhs[0])!=3) || (mxGetM(prhs[0])!=1) || (!mxIsClass(prhs[0],"double")) )
    mexErrMsgTxt("First input must be a 1-by-3 matrix of type double");
if ( (mxGetN(prhs[1])!=6) || (mxGetM(prhs[1])!=1) || (!mxIsClass(prhs[1],"double")) )
    mexErrMsgTxt("Second input must be a 1-by-6 matrix of type double");
if ( (mxGetN(prhs[2])!=6) || (mxGetM(prhs[2])!=1) || (!mxIsClass(prhs[2],"double")) )
    mexErrMsgTxt("Third input must be a 1-by-6 matrix of type double");

double const *gridSize, *gridBounds, *lineCoord;

gridSize = mxGetPr(prhs[0]); // [Nx Ny Nz]
gridBounds = mxGetPr(prhs[1]); // [xMin yMin zMin xMax yMax zMax],
lineCoord = mxGetPr(prhs[2]);// [x1 y1 z1 x2 y2 z2],

double voxelSizeX, voxelSizeY, voxelSizeZ;
double tDeltaX, tDeltaY, tDeltaZ; // parametric step size along different dimensions
double tMaxX, tMaxY, tMaxZ; // used to determine the dimension of the next step along the line
double tMin, tMax; // maximum and minimum parameteric coordinates
int X, Y, Z, Xend, Yend, Zend; // voxel subscripts
int stepX, stepY, stepZ; // direction of grid traversal
int storageSize; //initial storage size - dynamically updated
int *intersectedVoxelData; //storage for the indexes of the intersected voxels - dynamically allocated
int addedVoxelCount;

double xStart, yStart, zStart, xEnd, yEnd, zEnd; // point where segment intersects the grid
double xVec, yVec, zVec; //direction of the line segment

// Intersection test, find minimum and maximum parameteric intersection coordinate
int intersectTest = boxIntersectTest(gridSize, gridBounds, lineCoord, tMin, tMax);
if (intersectTest==0) {
	//return empty array
	plhs[0] = mxCreateNumericMatrix(0,0,mxINT32_CLASS,mxREAL);
	return;
}
tMin = MAX(tMin,0);
tMax = MIN(tMax,1);
// Compute helpful variables
voxelSizeX = (gridBounds[3] - gridBounds[0])/gridSize[0];
voxelSizeY = (gridBounds[4] - gridBounds[1])/gridSize[1];
voxelSizeZ = (gridBounds[5] - gridBounds[2])/gridSize[2];
xVec = lineCoord[3] - lineCoord[0];
yVec = lineCoord[4] - lineCoord[1];
zVec = lineCoord[5] - lineCoord[2];
xStart = lineCoord[0] + xVec*tMin;
yStart = lineCoord[1] + yVec*tMin;
zStart = lineCoord[2] + zVec*tMin;
xEnd = lineCoord[0] + xVec*tMax;
yEnd = lineCoord[1] + yVec*tMax;
zEnd = lineCoord[2] + zVec*tMax;

// Allocate memory to store the indexes of the intersected voxels
storageSize = gridSize[0] + gridSize[1] + gridSize[2]; 
plhs[0] = mxCreateNumericMatrix(storageSize,1,mxINT32_CLASS,mxREAL); // initializes to zeros
intersectedVoxelData = (int*)mxGetData(plhs[0]);

// Determine initial voxel coordinates and line traversal directions
// X-dimension
X = MAX(1,ceil((xStart-gridBounds[0])/voxelSizeX));  // starting coordinate - include left boundary - index starts from 1
Xend = MAX(1,ceil((xEnd-gridBounds[0])/voxelSizeX)); // ending coordinate - stepping continues until we hit this index
if (xVec>0) 
{
    stepX = 1;
    tDeltaX = voxelSizeX/xVec; //parametric step length between the x-grid planes
    tMaxX = tMin + (gridBounds[0] + X*voxelSizeX - xStart)/xVec; // parametric distance until the first crossing with x-grid plane
}
else if (xVec<0)
{
    stepX = -1;
    tDeltaX = voxelSizeX/-xVec; //parametric step length between the x-grid planes
    tMaxX = tMin + (gridBounds[0] + (X-1)*voxelSizeX - xStart)/xVec; // parametric distance until the first crossing with x-grid plane
}
else
{
    stepX = 0;
    tMaxX = tMax; // the line doesn't cross the next x-plane
    tDeltaX = tMax; // set the parametric step to maximum
}
// Y-dimension
Y = MAX(1,ceil((yStart-gridBounds[1])/voxelSizeY));
Yend = MAX(1,ceil((yEnd-gridBounds[1])/voxelSizeY));
if (yVec>0) 
{
    stepY = 1;
    tDeltaY = voxelSizeY/yVec;
    tMaxY = tMin + (gridBounds[1] + Y*voxelSizeY - yStart)/yVec;
}
else if (yVec<0)
{
    stepY = -1;
    tDeltaY = voxelSizeY/-yVec;
    tMaxY = tMin + (gridBounds[1] + (Y-1)*voxelSizeY - yStart)/yVec;
}
else
{
    stepY = 0;
    tMaxY = tMax;
    tDeltaY = tMax;
}
// Z-dimension
Z = MAX(1,ceil((zStart-gridBounds[2])/voxelSizeZ));
Zend = MAX(1,ceil((zEnd-gridBounds[2])/voxelSizeZ));
if (zVec>0) 
{
    stepZ = 1;
    tDeltaZ = voxelSizeZ/zVec;
    tMaxZ = tMin + (gridBounds[2] + Z*voxelSizeZ - zStart)/zVec;
}
else if (zVec<0)
{
    stepZ = -1;
    tDeltaZ = voxelSizeZ/-zVec;
    tMaxZ = tMin + (gridBounds[2] + (Z-1)*voxelSizeZ - zStart)/zVec;
}
else
{
    stepZ = 0;
    tMaxZ = tMax;
    tDeltaZ = tMax;
}

// Add initial voxel to the list
intersectedVoxelData[0] = sub2ind(X, Y, Z, gridSize);
addedVoxelCount = 1;
// Step iteratively through the grid
while ((X!=Xend)||(Y!=Yend)||(Z!=Zend))
{
    if (tMaxX<tMaxY)
    {
        if (tMaxX<tMaxZ)
        {
            X += stepX;
            tMaxX += tDeltaX;
        }
        else
        {
            Z += stepZ;
            tMaxZ += tDeltaZ;
        }
    }
    else
    {
        if (tMaxY<tMaxZ)
        {
            Y += stepY;
            tMaxY += tDeltaY;
        }
        else
        {
            Z += stepZ;
            tMaxZ += tDeltaZ;
        }
    }
    addedVoxelCount++;
    //must perform memory check - if the initial allocated array is large enough this step is not necessary
    if (addedVoxelCount>storageSize)
    {
        storageSize = storageSize*2;
        intersectedVoxelData = (int*)mxRealloc(intersectedVoxelData, sizeof(int)*storageSize);
    }
    intersectedVoxelData[addedVoxelCount-1] = sub2ind(X, Y, Z, gridSize);
}
// Update the size of the output matrix
intersectedVoxelData = (int*)mxRealloc(intersectedVoxelData, sizeof(int)*addedVoxelCount);
mxSetM(plhs[0], addedVoxelCount); //number of rows
mxSetN(plhs[0], 1); //number of columns
mxSetData(plhs[0], intersectedVoxelData); // update pointer to the matrix data
}

int sub2ind(const int X, const int Y, const int Z, const double * gridSize)
{
	return (Y + (X - 1)*gridSize[0] + (Z - 1)*gridSize[1]*gridSize[0]);
}

int boxIntersectTest(double const *gridSize, double const *gridBounds, double const *lineCoord, double & tMinR, double & tMaxR)
{
double tMin, tMax, tYMin, tYMax, tZMin, tZMax;
double divX, divY, divZ;

divX = 1/(lineCoord[3] - lineCoord[0]);

if (divX >= 0) // t-coordinate of box bounds
{
     tMin = (gridBounds[0] - lineCoord[0])*divX;
     tMax = (gridBounds[3] - lineCoord[0])*divX;
} 
else 
{
     tMin = (gridBounds[3] - lineCoord[0])*divX;
     tMax = (gridBounds[0] - lineCoord[0])*divX;
}

divY = 1/(lineCoord[4] - lineCoord[1]);

if (divY >= 0) 
{
     tYMin = (gridBounds[1] - lineCoord[1])*divY;
     tYMax = (gridBounds[4] - lineCoord[1])*divY;
} 
else 
{
     tYMin = (gridBounds[4] - lineCoord[1])*divY;
     tYMax = (gridBounds[1] - lineCoord[1])*divY;
}

if ( (tMin > tYMax) || (tYMin > tMax) ) // check if line misses the box
	return false;
if (tYMin > tMin)
	tMin = tYMin;
if (tYMax < tMax)
	tMax = tYMax;

divZ = 1/(lineCoord[5] - lineCoord[2]);

if (divZ >= 0) 
{
     tZMin = (gridBounds[2] - lineCoord[2])*divZ;
     tZMax = (gridBounds[5] - lineCoord[2])*divZ;
} 
else 
{
     tZMin = (gridBounds[5] - lineCoord[2])*divZ;
     tZMax = (gridBounds[2] - lineCoord[2])*divZ;
}

if ((tMin > tZMax) || (tZMin > tMax)) // check if line misses the box
	return false;
if (tZMin > tMin)
	tMin = tZMin;
if (tZMax < tMax)
	tMax = tZMax;
if ((tMin>=1)&&(tMax>=1))
    return false;
if ((tMin<=0)&&(tMax<=0))
    return false;
tMinR = tMin;
tMaxR = tMax;
return 1;
}


import gab.opencv.*;
import SimpleOpenNI.*;

SimpleOpenNI kinect;
OpenCV opencv;
PImage depthCam;
PImage colorCam;
PImage result;
PImage binary_result;
ArrayList<PVector> contours;
ArrayList<PVector> interiors;
ArrayList<PVector> sortedContours;
FloatList minDistances;
float closestX = 0;
float closestY = 0;
float previousX;
float previousY;

void setup()
{
  size(640, 480);
  background(0);
  frameRate(60);
  kinect  = new SimpleOpenNI(this);
  kinect.enableDepth();
  opencv = new OpenCV(this, 640, 480);
  result = createImage(width, height, RGB);
  binary_result = createImage(width, height, ALPHA);
  contours = new ArrayList<PVector>();
  interiors = new ArrayList<PVector>();
  sortedContours = new ArrayList<PVector>();
  minDistances = new FloatList();
}
 
void draw()
{
  background(0);
  kinect.update();
  contours.clear();
  interiors.clear();
  sortedContours.clear();
  depthCam = kinect.depthImage();
  int [] depthValues = kinect.depthMap();
  binary_result.loadPixels();
  result.loadPixels();
  float sumX = 0;
  float sumY = 0;
  float count = 0;
  int currentMin = 8000;
  for(int i = 0; i < depthValues.length; i++){
    if(depthValues[i] < currentMin && depthValues[i] > 610){
      currentMin = depthValues[i];
    }
  }
  int currentMax = currentMin + 100;
  for(int y = 0; y<depthCam.height; y++){
    for(int x = 0; x<depthCam.width; x++){
      int loc = x + y*depthCam.width;
      if(depthValues[loc] > currentMin && depthValues[loc] < currentMax){
        int upLoc;
        if((loc - depthCam.width) >= 0) { upLoc = loc - depthCam.width; } else {upLoc = loc;}
        int downLoc;
        if ((loc + depthCam.width) < depthValues.length) {downLoc = loc + depthCam.width;} else {downLoc = loc;}
        int leftLoc;
        if ((loc - 1) >= 0) { leftLoc = loc - 1; } else {leftLoc = loc;}
        int rightLoc;
        if ((loc + 1) < depthValues.length){ rightLoc = loc + 1; } else {rightLoc = loc;}
        
        sumX += x;
        sumY += y;
        count++;
        binary_result.pixels[loc] = color(1);
        if((depthValues[upLoc] > currentMin && depthValues[upLoc] < currentMax) && (depthValues[downLoc] > currentMin && depthValues[downLoc] < currentMax) && (depthValues[leftLoc] > currentMin && depthValues[leftLoc] < currentMax) && (depthValues[rightLoc] > currentMin && depthValues[rightLoc] < currentMax)){
          result.pixels[loc] = color(255);
          PVector interiorPoint = new PVector(x,y);
          interiors.add(interiorPoint);
        }else{
          //result.pixels[loc] = color(0, 0, 255);
          PVector contourPoint = new PVector(x,y);
          contours.add(contourPoint);
        }
      }else{
        result.pixels[loc] = color(0);
        binary_result.pixels[loc] = color(0);
      }
    }
  }
  
  binary_result.updatePixels();
  result.updatePixels();
  image(result,0,0);
  
  // To sort contours
  // *** Algorithm ***
  // If grey pixel (valid contour point) is hit, turn clockwise and forward by a pixel
  // else if white pixel is hit, repeat turning anticlockwise and forwarding by a pixel until a grey pixel is hit again
  // Do this until you get to the pixel you started from
  
  // Direction: 
  // (clockwise)
  // 0 - right
  // 1 - down
  // 2 - left
  // 3 - up
  int dir = 0;
  
  // Locate starting contour point
  int startLoc = -1;
  int xLoc = -1;
  int yLoc = -1;
  for(int y = 0; y < depthCam.height; y++){
    for(int x = 0; x < depthCam.width; x++){
      int loc = x + y*depthCam.width;
      if(binary_result.pixels[loc] == color(1) && startLoc == -1){
        startLoc = loc;
        xLoc = x;
        yLoc = y;
        //sortedContours.add(new PVector(xLoc, yLoc));
      }else{
        continue;
      }
    }
  }
  
  int currLoc = 0;
  if(startLoc >= 0) currLoc = startLoc;
  do {
    if(binary_result.pixels[currLoc] == color(1)){
      dir = (dir + 4 - 1) % 4; 
      sortedContours.add(new PVector(xLoc, yLoc));
    }else if(binary_result.pixels[currLoc] == color(0)){
      dir = (dir + 1) % 4;
    }
    
    switch(dir) {
      case 0:
      currLoc = currLoc + 1;
      xLoc++;
      break;
      case 1:
      currLoc = currLoc + depthCam.width;
      yLoc++;
      break;
      case 2:
      currLoc = currLoc - 1;
      xLoc--;
      break;
      case 3: 
      currLoc = currLoc - depthCam.width;
      yLoc--;
      break;
    }
    
    if(currLoc < 0 || currLoc >= depthCam.width * depthCam.height) currLoc = 0;
    
  } while(currLoc != startLoc);

  // To find center of palm
  /*
  minDistances.clear();
  for(int i = 0; i < interiors.size(); i++){
    minDistances.append(0);
  }
  for(int i = 0; i < interiors.size(); i++){
    float minDist = 800;
    for(int j = 0; j < contours.size(); j++){
      float distance = interiors.get(i).dist(contours.get(j));
      if( distance < minDist ){
        minDist = distance;
        minDistances.set(i, distance);
      }
    }
  }
  float maxDist = 0;
  int maxIndex = 0;
  for(int i = 0; i < minDistances.size(); i++){
    if(minDistances.get(i) > maxDist){
      maxDist = minDistances.get(i);
      maxIndex = i;
    }
  }
  fill(255,255,0);
  if(interiors.size() != 0) ellipse(interiors.get(maxIndex).x, interiors.get(maxIndex).y, 20, 20);
  */
  
  
  // Simpler way to find center of palm
  PVector center = new PVector(0, 0);
  if(count != 0) {
    float avgX = sumX / count;
    float avgY = sumY / count;
    center.set(avgX, avgY);
    fill(255, 0, 0);
    ellipse(avgX, avgY, 16, 16);
  }
  
  // To find fingertips
  if(sortedContours.size() > 800){
    for(int i = 0; i < sortedContours.size(); i++){
      int k = 22;
      float angle = 40;
      PVector curr = sortedContours.get(i);
      PVector prev, next;
      if ((i-k) >= 0) { prev = sortedContours.get(i-k);} else { prev = sortedContours.get(sortedContours.size() + i - k);}
      if ((i+k) < sortedContours.size()) { next = sortedContours.get(i+k); } else { next = sortedContours.get(i + k - sortedContours.size());}
      PVector one = PVector.sub(prev, curr);
      PVector two = PVector.sub(next, curr);
      float a = PVector.angleBetween(one, two);
      a = degrees(a);
      if(a < angle && a != 0){
        PVector midPt = PVector.lerp(prev, next, 0.5);
        float distPi = PVector.dist(center, curr);
        float distPik = PVector.dist(center, midPt);
        if(distPi > distPik){
          fill(255, 255, 0);
          ellipse(curr.x, curr.y, 16, 16);
          i += sortedContours.size() * 0.09;
        }
      }
    }
  }

  
  
}

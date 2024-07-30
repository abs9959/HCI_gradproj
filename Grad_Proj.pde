
/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
//import org.json.JSONObject;
import processing.data.JSONObject;
import processing.data.JSONArray;
import java.util.ArrayList;

/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;


int               hardwareVersion                     = 3;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 

/* elements definition *************************************************************************************************/

HVirtualCoupling  s;

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 400.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* forces and radial params */
PVector           actingForce                         = new PVector(0, 0);

float             min_x                               = -0.07;
float             max_x                               = 0.055;
float             min_y                               = 0.;
float             max_y                               = 0.1;

//scaling factors for tangential force application for height based forces
float scalingFactorX = 2;
float scalingFactorY = 2;


float[] xPositions = {-0.04, 0.0, 0.04}; // Adjust positions along x plane
float[] yPositions = {0.03, 0.05, 0.07}; // Adjust positions along y plane


 float endEffectorX;
float endEffectorY; 
 
 
  // Define the distance threshold for applying force
  double threshold = 2;
  float forceMagnitude = 1.0;

  

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           mappedEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
FWorld            world;
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;


/* graphical elements */
PShape pGraph, joint, endEffector;
PShape wall;

/* Lab3 variables */
PVector lastPosEE = new PVector(0,0);
enum FFNUM {
  ONE,
  TWO,
  THREE,
  FOUR,
  FIVE,
  SIX,
  SEVEN
};
FFNUM ffNum = FFNUM.ONE;
PFont f;


PImage textureImage, textureImage2, textureImage3, textureImage4, textureImage5, flower;

ArrayList<LineSegment> lineSegments1 = new ArrayList<LineSegment>();
ArrayList<LineSegment> lineSegments2 = new ArrayList<LineSegment>();
ArrayList<LineSegment> lineSegments3 = new ArrayList<LineSegment>();
ArrayList<LineSegment> lineSegments4 = new ArrayList<LineSegment>();
ArrayList<LineSegment> lineSegments5 = new ArrayList<LineSegment>();


//define line segment class
class LineSegment {
  PVector start;
  PVector end;
  PVector normal;

  LineSegment(PVector start, PVector end, PVector normal) {
    this.start = start;
    this.end = end;
    this.normal = normal;
  }
}

class FloatVector {

  float x;
  float y;

  FloatVector(float x_, float y_) {
    x = x_;
    y = y_;
  }

}



/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 700);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  
  /* device setup */

  haplyBoard          = new Board(this, "COM5", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();

  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);

  flower = loadImage("flower.jpg");

  
  textureImage = loadImage("line1.jpg");
  String jsonString = join(loadStrings("normals1.json"), "");
  JSONArray jsonArray = JSONArray.parse(jsonString);
  
  println("made it");

  for (int i = 0; i < jsonArray.size(); i++) {
    
    JSONObject obj = jsonArray.getJSONObject(i);
    JSONObject line = obj.getJSONObject("line");
    JSONObject normal = obj.getJSONObject("normal");

    PVector start = new PVector(line.getFloat("x1"), line.getFloat("y1"));
    PVector end = new PVector(line.getFloat("x2"), line.getFloat("y2"));
    PVector norm = new PVector(normal.getFloat("nx"), normal.getFloat("ny"));
    
    PVector transformedStart = transformPoint(start.x, start.y);
    PVector transformedEnd = transformPoint(end.x, end.y);

    lineSegments1.add(new LineSegment(transformedStart, transformedEnd, norm));
  }
  println("Done loading JSON1.");
  
    textureImage2 = loadImage("line2.jpg");
  String jsonString2 = join(loadStrings("normals2.json"), "");
  JSONArray jsonArray2 = JSONArray.parse(jsonString2);

  for (int i = 0; i < jsonArray2.size(); i++) {
    JSONObject obj = jsonArray2.getJSONObject(i);
    JSONObject line = obj.getJSONObject("line");
    JSONObject normal = obj.getJSONObject("normal");

    PVector start = new PVector(line.getFloat("x1"), line.getFloat("y1"));
    PVector end = new PVector(line.getFloat("x2"), line.getFloat("y2"));
    PVector norm = new PVector(normal.getFloat("nx"), normal.getFloat("ny"));
    
    PVector transformedStart = transformPoint2(start.x, start.y);
    PVector transformedEnd = transformPoint2(end.x, end.y);

    lineSegments2.add(new LineSegment(transformedStart, transformedEnd, norm));
  }
  println("Done loading JSON2.");
  
      textureImage3 = loadImage("IMG_3505.jpg");
  String jsonString3 = join(loadStrings("normals3NEW.json"), "");
  JSONArray jsonArray3 = JSONArray.parse(jsonString3);

  for (int i = 0; i < jsonArray3.size(); i++) {
    JSONObject obj = jsonArray3.getJSONObject(i);
    JSONObject line = obj.getJSONObject("line");
    JSONObject normal = obj.getJSONObject("normal");

    PVector start = new PVector(line.getFloat("x1"), line.getFloat("y1"));
    PVector end = new PVector(line.getFloat("x2"), line.getFloat("y2"));
    PVector norm = new PVector(normal.getFloat("nx"), normal.getFloat("ny"));
    
    PVector transformedStart = transformPoint2(start.x, start.y);
    PVector transformedEnd = transformPoint2(end.x, end.y);

    lineSegments3.add(new LineSegment(transformedStart, transformedEnd, norm));
  }
  println("Done loading JSON3.");
  
        textureImage4 = loadImage("IMG_3499.jpg");
  String jsonString4 = join(loadStrings("normalsTEST2.json"), "");
  JSONArray jsonArray4 = JSONArray.parse(jsonString4);

  for (int i = 0; i < jsonArray4.size(); i++) {
    JSONObject obj = jsonArray4.getJSONObject(i);
    JSONObject line = obj.getJSONObject("line");
    JSONObject normal = obj.getJSONObject("normal");

    PVector start = new PVector(line.getFloat("x1"), line.getFloat("y1"));
    PVector end = new PVector(line.getFloat("x2"), line.getFloat("y2"));
    PVector norm = new PVector(normal.getFloat("nx"), normal.getFloat("ny"));
    
    PVector transformedStart = transformPoint2(start.x, start.y);
    PVector transformedEnd = transformPoint2(end.x, end.y);

    lineSegments4.add(new LineSegment(transformedStart, transformedEnd, norm));
  }
  println("Done loading JSON4.");
  
          textureImage5 = loadImage("IMG_35011.jpg");
  String jsonString5 = join(loadStrings("normals5.json"), "");
  JSONArray jsonArray5 = JSONArray.parse(jsonString5);

  for (int i = 0; i < jsonArray5.size(); i++) {
    JSONObject obj = jsonArray5.getJSONObject(i);
    JSONObject line = obj.getJSONObject("line");
    JSONObject normal = obj.getJSONObject("normal");

    PVector start = new PVector(line.getFloat("x1"), line.getFloat("y1"));
    PVector end = new PVector(line.getFloat("x2"), line.getFloat("y2"));
    PVector norm = new PVector(normal.getFloat("nx"), normal.getFloat("ny"));
    
    PVector transformedStart = transformPoint2(start.x, start.y);
    PVector transformedEnd = transformPoint2(end.x, end.y);

    lineSegments5.add(new LineSegment(transformedStart, transformedEnd, norm));
  }
  println("Done loading JSON5.");
  

}
/* end setup section ***************************************************************************************************/
  
/* draw section ********************************************************************************************************/
void draw(){


  
  if(!renderingForce) {
    background(255);
    
    if(ffNum != FFNUM.TWO && ffNum != FFNUM.FOUR && ffNum != FFNUM.THREE && ffNum != FFNUM.FIVE && ffNum != FFNUM.SIX && ffNum != FFNUM.SEVEN) {
    text("*Haptic Experience Window*", 100, 75);
    fill(#000000);
    text("Instructions:\nEach of the 5 experiences below represents a texture. \nThe textures are as follows:\n1. Rough \n2. Large bumps \n3. Harsh edges \n4. Rough (less granular) \n5. Initial exploration of tracing a curved line \nMake sure the mouse is focussed on the Haptic Experience Window. \nPress '1' for the first experience. \nPress '2' for the second experience and so on... ", 100, 100);
    fill(#000000);
    text("Current mode:", 300, 300);
    fill(#000000);
    if(ffNum == FFNUM.ONE) {
      text("First mode", 500, 300);
    } else if(ffNum == FFNUM.TWO) {
      text("Second mode", 500, 300);
    } else if(ffNum == FFNUM.FIVE) {
      text("Fifth mode", 500, 300);
    } else if(ffNum == FFNUM.FOUR) {
      text("Fourth mode", 500, 300);
    } else if(ffNum == FFNUM.SIX) {
      text("Sixth mode", 500, 300);
    } else {
      text("Third mode", 500, 300);
    }
    
    ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);

    
  }
  
    // Show end effector marker and hide initial text when in bump environment

 
  
  
  // Show end effector marker and hide initial text when in bump environment
    if(ffNum == FFNUM.SIX) {
      float scaleFactor = 0.2; // Adjust as needed
    
      // Calculate the scaled width and height of the image
      float scaledWidth = textureImage5.width * scaleFactor;
      float scaledHeight = textureImage5.height * scaleFactor;
    
      image(textureImage5, 200, 80, scaledWidth, scaledHeight);
      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
    
      // Draw line segments
      //for (LineSegment segment : lineSegments5) {
      //  stroke(255, 0, 0);
      //  line(segment.start.x, segment.start.y, segment.end.x, segment.end.y);
      //}
   

  }
  
    if(ffNum == FFNUM.SEVEN) {
    float scaleFactor = 0.6; // Adjust as needed
    
    // Calculate the scaled width and height of the image
    float scaledWidth = flower.width * scaleFactor;
    float scaledHeight = flower.height * scaleFactor;
    
    image(flower, map(-0.05,-0.079, 0.071, 0, 1000), map(0.01, 0, 0.1, 0, 700), scaledWidth, scaledHeight);

    
    
    ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
   

  }
  
  
  if(ffNum == FFNUM.TWO) {
      float scaleFactor = 0.2; // Adjust as needed
    
      // Calculate the scaled width and height of the image
      float scaledWidth = textureImage4.width * scaleFactor;
      float scaledHeight = textureImage4.height * scaleFactor;
    
      image(textureImage4, 200, 80, scaledWidth, scaledHeight);
      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
    
      // Draw line segments
      //for (LineSegment segment : lineSegments4) {
      //  stroke(255, 0, 0);
      //  line(segment.start.x, segment.start.y, segment.end.x, segment.end.y);
      //}
   

  }
  
  if(ffNum == FFNUM.THREE) {

      float scaleFactor = 0.2; // Adjust as needed
    
      // Calculate the scaled width and height of the image
      float scaledWidth = textureImage3.width * scaleFactor;
      float scaledHeight = textureImage3.height * scaleFactor;
    
      image(textureImage3, 200, 80, scaledWidth, scaledHeight);
      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
    
      // Draw line segments
      //for (LineSegment segment : lineSegments3) {
      //  stroke(255, 0, 0);
      //  line(segment.start.x, segment.start.y, segment.end.x, segment.end.y);
      //}
    
  }
  
    if(ffNum == FFNUM.FOUR) {
      
      float scaleFactor = 0.2; // Adjust as needed
    
      // Calculate the scaled width and height of the image
      float scaledWidth = textureImage2.width * scaleFactor;
      float scaledHeight = textureImage2.height * scaleFactor;
    
      image(textureImage2, 200, 80, scaledWidth, scaledHeight);
      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
    
      // Draw line segments
      //for (LineSegment segment : lineSegments2) {
      //  stroke(255, 0, 0);
      //  line(segment.start.x, segment.start.y, segment.end.x, segment.end.y);
      //}
    
   
  }
  
  if (ffNum == FFNUM.FIVE) {

   
    float scaleFactor = 0.78; // Adjust as needed
    
    // Calculate the scaled width and height of the image
    float scaledWidth = textureImage.width * scaleFactor;
    float scaledHeight = textureImage.height * scaleFactor;
    
    //image(textureImage, map(-0.05,-0.079, 0.071, 0, 1000), map(0.01, 0, 0.1, 0, 700), scaledWidth, scaledHeight);
    image(textureImage, 200, 80, scaledWidth, scaledHeight);
    //image(textureImage, 10, 140, scaledWidth, scaledHeight);

    ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
    
  //// Draw line segments
  //for (LineSegment segment : lineSegments1) {
  //  //PVector transformedStart = transformPoint(segment.start.x, segment.start.y);
  //  //PVector transformedEnd = transformPoint(segment.end.x, segment.end.y);
    
  //  stroke(255, 0, 0);
  //  //line(transformedStart.x, transformedStart.y, transformedEnd.x, transformedEnd.y);
  //  line(segment.start.x, segment.start.y, segment.end.x, segment.end.y);
  //}
    
    
    
    

  }
 
    
    
  }
}
/* end draw section ****************************************************************************************************/

  
/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      lastPosEE = posEE.copy();

      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(device_to_graphics(posEE)); 
      
      
      
      /* haptic wall force calculation */
      actingForce.set(0, 0);
 /* SNAG GRID  **************************************************************************************************/ 
      if (ffNum == FFNUM.ONE) {
          //fEE.set(10,0);
      }

else if (ffNum == FFNUM.SEVEN) {
    if(posEE.y > 0.0283 && posEE.y <= 0.0301){
        if(posEE.x > -0.0185 && posEE.x <= -0.004){
          if(posEE.y > 0.0282 && posEE.y <= 0.0295) {
          fEE.set(0,-3);
          }
        }
      }
  //top of right    
      else if(posEE.y > 0.0363 && posEE.y <= 0.0383){
        if(posEE.x > -0.0102 && posEE.x <= 0.002){
          if(posEE.y > 0.0363 && posEE.y <= 0.0372) {
          fEE.set(0,-3);
          }
        }
        
        if(posEE.x > -0.0186 && posEE.x <= -0.0172){
        if(posEE.y > 0.0293 && posEE.y <= 0.0429){
          if(posEE.x > -0.0186 && posEE.x <= -0.0177) {
          fEE.set(-3,0);
          }
        }
      }
        
        
      }
      
      else if(posEE.y > 0.0499 && posEE.y <= 0.0520){
        if(posEE.x > -0.0047 && posEE.x <= 0.0031){
          if(posEE.y > 0.0499 && posEE.y <= 0.0507) {
          fEE.set(0,3);
          }
        }
      }
      
      else if(posEE.y > 0.0635 && posEE.y <= 0.0652){
        if(posEE.x > -0.0177 && posEE.x <= -0.0041){
          if(posEE.y > 0.0635 && posEE.y <= 0.0642) {
          fEE.set(0,3);
          }
        }
      }
      
       else if(posEE.y > 0.0590 && posEE.y <= 0.0605){
        if(posEE.x > -0.0261 && posEE.x <= -0.0133){
          if(posEE.y > 0.0590 && posEE.y <= 0.0595) {
          fEE.set(0,3);
          }
        }
      }
 //top of bottom left     
       else if(posEE.y > 0.0440 && posEE.y <= 0.0461){
        if(posEE.x > -0.0257 && posEE.x <= -0.0130){
          if(posEE.y > 0.0440 && posEE.y <= 0.0453) {
          fEE.set(0,-3);
          }
        }
        
        if(posEE.x > -0.0293 && posEE.x <= -0.0277){
        if(posEE.y > 0.0329 && posEE.y <= 0.0474){
          if(posEE.x > -0.0293 && posEE.x <= -0.0285) {
          fEE.set(-3,0);
          }
        }
      }
      
      if(posEE.x > 0.0019 && posEE.x <= 0.0033){
        if(posEE.y > 0.0370 && posEE.y <= 0.0503){
          if(posEE.x > 0.0019 && posEE.x <= 0.0026) {
          fEE.set(3,0);
          }
        }
      }
      
      if(posEE.x > -0.0102 && posEE.x <= -0.0086){
        if(posEE.y > 0.0374 && posEE.y <= 0.0499){
          if(posEE.x > -0.0102 && posEE.x <= -0.0094) {
          fEE.set(-3,0);
          }
        }
      }
        
        
      }
   //top of top left   
       else if(posEE.y > 0.0320 && posEE.y <= 0.0337){
        if(posEE.x > -0.0284 && posEE.x <= -0.0179){
          if(posEE.y > 0.0320 && posEE.y <= 0.0329) {
          fEE.set(0,-3);
          }
        }
        
        if(posEE.x > -0.0053 && posEE.x <= -0.0038){
        println("1");
        if(posEE.y > 0.0280 && posEE.y <= 0.0372){
          println("2");
          if(posEE.x > -0.0055 && posEE.x <= -0.0046) {
            println("3");
          fEE.set(3,0);
          }
        }
      }
        
        
      }
   //bottom of top   
      else if(posEE.y > 0.0419 && posEE.y <= 0.0436){
        if(posEE.x > -0.0174 && posEE.x <= -0.0095){
          if(posEE.y > 0.0419 && posEE.y <= 0.0427) {
          fEE.set(0,3);
          }
        }
        
        if(posEE.x > -0.0293 && posEE.x <= -0.0277){
        if(posEE.y > 0.0329 && posEE.y <= 0.0474){
          if(posEE.x > -0.0293 && posEE.x <= -0.0285) {
          fEE.set(-3,0);
          }
        }
      }
      
      if(posEE.x > 0.0019 && posEE.x <= 0.0033){
        if(posEE.y > 0.0370 && posEE.y <= 0.0503){
          if(posEE.x > 0.0019 && posEE.x <= 0.0026) {
          fEE.set(3,0);
          }
        }
      }
        
      }
      
      else if(posEE.y > 0.0489 && posEE.y <= 0.0506){
        if(posEE.x > -0.0132 && posEE.x <= -0.0050){
          if(posEE.y > 0.0489 && posEE.y <= 0.0497) {
          fEE.set(0,-3);
          }
        }
        
        if(posEE.x > -0.0262 && posEE.x <= -0.0249){
        if(posEE.y > 0.0462 && posEE.y <= 0.0599){
          if(posEE.x > -0.0262 && posEE.x <= 0.0258) {
          fEE.set(-3,0);
          }
        }
      }
      }
//right of top      
      else if(posEE.x > -0.0053 && posEE.x <= -0.0038){
        println("1");
        if(posEE.y > 0.0280 && posEE.y <= 0.0372){
          println("2");
          if(posEE.x > -0.0055 && posEE.x <= -0.0046) {
            println("3");
          fEE.set(3,0);
          }
        }
        if(posEE.y > 0.0488 && posEE.y <= 0.0646){
          if(posEE.x > -0.0053 && posEE.x <= -0.0045) {
          fEE.set(3,0);
          }
        }
      }
   //right of right   
      else if(posEE.x > 0.0019 && posEE.x <= 0.0033){
        if(posEE.y > 0.0370 && posEE.y <= 0.0503){
          if(posEE.x > 0.0019 && posEE.x <= 0.0026) {
          fEE.set(3,0);
          }
        }
      }
 //right of bottom left     
      else if(posEE.x > -0.0138 && posEE.x <= -0.0124){
        println("I AM HERE");
        if(posEE.y > 0.0446 && posEE.y <= 0.0599){
          if(posEE.x > -0.01389 && posEE.x <= 0.0026) {
          fEE.set(3,0);
          }
          
        }
      }
 //left of bottom left     
      else if(posEE.x > -0.0262 && posEE.x <= -0.0249){
        if(posEE.y > 0.0462 && posEE.y <= 0.0599){
          if(posEE.x > -0.0262 && posEE.x <= 0.0258) {
          fEE.set(-3,0);
          }
        }
      }
 //left of left     
      else if(posEE.x > -0.0293 && posEE.x <= -0.0277){
        if(posEE.y > 0.0329 && posEE.y <= 0.0474){
          if(posEE.x > -0.0293 && posEE.x <= -0.0285) {
          fEE.set(-3,0);
          }
        }
      }
 // left of top    
      else if(posEE.x > -0.0186 && posEE.x <= -0.0172){
        if(posEE.y > 0.0293 && posEE.y <= 0.0429){
          if(posEE.x > -0.0186 && posEE.x <= -0.0177) {
          fEE.set(-3,0);
          }
        }
      }
      
 // left of right    
      else if(posEE.x > -0.0102 && posEE.x <= -0.0086){
        if(posEE.y > 0.0374 && posEE.y <= 0.0499){
          if(posEE.x > -0.0102 && posEE.x <= -0.0094) {
          fEE.set(-3,0);
          }
        }
      }

      
      
      else if(posEE.y > 0.0780 && posEE.y < 0.0975){
        if(posEE.x > -0.0438 && posEE.x < 0.0172){
          
          
          float scale = 3;
          float perlin_noise = noise(posEE.x, posEE.y);
          float magnitude = 5 + perlin_noise * scale + random(-1, 1);
          float direction = perlin_noise * TWO_PI + random(-PI/4, PI/4);
          
          // Create a force vector based on magnitude and direction obtained from the Perlin noise
          float forceX = magnitude * cos(direction);
          float forceY = magnitude * sin(direction);
          PVector forceVector = new PVector(forceX, forceY);
          println(forceVector);
          fEE.set(forceVector);

          
          /*
          double threshold = 0.00025; // Threshold for distance from multiples of 0.1
          double interval = 0.001;    // Interval between multiples of 0.1
          
          // Calculate the nearest multiple of 0.1 to posEE.x
          double nearestMultipleX = Math.round(posEE.x / interval) * interval;
          
          // Calculate the distance between posEE.x and the nearest multiple of 0.1
          double distanceX = Math.abs(posEE.x - nearestMultipleX);
          
          // Check if the distance is within the threshold
          if (distanceX <= threshold) {
              fEE.set(75, 0);
          } else {
              fEE.set(0, 0);
          }
          
          // Calculate the nearest multiple of 0.1 to posEE.x
          double nearestMultipleY = Math.round(posEE.y / interval) * interval;
          
          // Calculate the distance between posEE.x and the nearest multiple of 0.1
          double distanceY = Math.abs(posEE.y - nearestMultipleY);
          
          // Check if the distance is within the threshold
          if (distanceY <= threshold) {
              fEE.set(0, 75);
          } else {
              fEE.set(0, 0);
          }*/
        
        }
      }
      
      else {
        fEE.set(0, 0); // No force if end-effector is not close to the polynomial function
      }
}


/* Large bumps  **************************************************************************************************/      
else if (ffNum == FFNUM.SIX) {
  
  PVector mappedEE = transformEE(posEE.x, posEE.y);
//println(mappedEE);

PVector appliedForce = new PVector(0, 0);
boolean forceApplied = false;

for (LineSegment ls : lineSegments5) {
  if (isNearLineSegment(ls, mappedEE)) {
    //println(mappedEE);
    PVector velocity = PVector.sub(posEE, lastPosEE);
    PVector forceDirection = ls.normal.copy().normalize();

    // Determine if the point is on the 'inside' or 'outside' of the line segment
    PVector toPoint = PVector.sub(mappedEE, ls.start);
    if (PVector.dot(toPoint, forceDirection) < 0) {
      forceDirection.mult(-1);
    }

    // Determine if the end effector is moving towards or away from the line segment
    if (PVector.dot(velocity, forceDirection) < 0) {
      // Moving towards the line segment: apply force to oppose motion (into shape)
      forceDirection.mult(-1);
    } else {
      // Moving away from the line segment: apply force to aid motion (out of shape)
      // Do nothing, forceDirection is already in the correct direction
    }

    PVector force = PVector.mult(forceDirection, forceMagnitude);
    appliedForce.add(force);
    forceApplied = true;
  }
}

if (!forceApplied) {
  appliedForce.set(0, 0);
}

fEE.set(appliedForce);

  
  
  
}  


/* Large bumps  **************************************************************************************************/      
else if (ffNum == FFNUM.TWO) {
  
  mappedEE = transformEE(posEE.x, posEE.y);
    //println(mappedEE);

 
  PVector appliedForce = new PVector(0, 0);
        boolean forceApplied = false;

        for (LineSegment ls : lineSegments4) {
          if (isNearLineSegment(ls, mappedEE)) {
            //println(mappedEE);
            PVector velocity = PVector.sub(posEE, lastPosEE);
            PVector forceDirection = ls.normal.copy().normalize();
            //PVector forceDirection = velocity.copy().normalize().mult(-1);

            // If velocity is in the same direction as the normal, reverse the force direction
            /*if (PVector.dot(velocity, forceDirection) > 0) {
              forceDirection.mult(-1);
            }*/
            
                // Determine if the point is on the 'inside' or 'outside' of the line segment
            PVector toPoint = PVector.sub(mappedEE, ls.start);
            if (PVector.dot(toPoint, forceDirection) < 0) {
              forceDirection.mult(-1);
            }
                    

            PVector force = PVector.mult(forceDirection, forceMagnitude);
            appliedForce.add(force);
            forceApplied = true;
          }
        }

        if (!forceApplied) {
          appliedForce.set(0, 0);
        }

        fEE.set(appliedForce);
  
  
  
}    
    
/* SNAG  **************************************************************************************************/      

   else if(ffNum == FFNUM.THREE) {
        mappedEE = transformEE(posEE.x, posEE.y);
    //println(mappedEE);

 
  PVector appliedForce = new PVector(0, 0);
        boolean forceApplied = false;

        for (LineSegment ls : lineSegments3) {
          if (isNearLineSegment(ls, mappedEE)) {
            //println(mappedEE);
            PVector velocity = PVector.sub(posEE, lastPosEE);
            PVector forceDirection = ls.normal.copy().normalize();
            //PVector forceDirection = velocity.copy().normalize().mult(-1);

            // If velocity is in the same direction as the normal, reverse the force direction
            /*if (PVector.dot(velocity, forceDirection) > 0) {
              forceDirection.mult(-1);
            }*/
            
                // Determine if the point is on the 'inside' or 'outside' of the line segment
            PVector toPoint = PVector.sub(mappedEE, ls.start);
            if (PVector.dot(toPoint, forceDirection) < 0) {
              forceDirection.mult(-1);
            }
                    

            PVector force = PVector.mult(forceDirection, forceMagnitude*2);
            appliedForce.add(force);
            forceApplied = true;
          }
        }

        if (!forceApplied) {
          appliedForce.set(0, 0);
        }

        fEE.set(appliedForce);
     
     
     
      } 
  
/* PYRAMID GRID  **************************************************************************************************/      
  else if(ffNum == FFNUM.FOUR)  {

    mappedEE = transformEE(posEE.x, posEE.y);
    //println(mappedEE);

 
  PVector appliedForce = new PVector(0, 0);
        boolean forceApplied = false;

        for (LineSegment ls : lineSegments2) {
          if (isNearLineSegment(ls, mappedEE)) {
            //println(mappedEE);
            PVector velocity = PVector.sub(posEE, lastPosEE);
            PVector forceDirection = ls.normal.copy().normalize();
            //PVector forceDirection = velocity.copy().normalize().mult(-1);

            // If velocity is in the same direction as the normal, reverse the force direction
            /*if (PVector.dot(velocity, forceDirection) > 0) {
              forceDirection.mult(-1);
            }*/
            
                // Determine if the point is on the 'inside' or 'outside' of the line segment
            PVector toPoint = PVector.sub(mappedEE, ls.start);
            if (PVector.dot(toPoint, forceDirection) < 0) {
              forceDirection.mult(-1);
            }
                    

            PVector force = PVector.mult(forceDirection, forceMagnitude);
            appliedForce.add(force);
            forceApplied = true;
          }
        }

        if (!forceApplied) {
          appliedForce.set(0, 0);
        }

        fEE.set(appliedForce);
    
    
    
  }
//CONTOUR FOLLOWING*********************************************************************************************************************

else if(ffNum == FFNUM.FIVE) {
  //println(lineSegments[1]);
  //println(posEE);
  mappedEE = transformEE(posEE.x, posEE.y);
    //mappedEE = transformPoint(posEE.x, posEE.y);
    //println(mappedEE);

  //mappedEE = posEE;
  
  //THIS WORKS
  /*println(mappedEE);
  //fEE.set(10,0);
  PVector appliedForce = new PVector(0, 0);
  boolean forceApplied = false;
  
  for (LineSegment ls : lineSegments) {
    
   if (isNearLineSegment(ls, mappedEE)) {
        PVector force = PVector.mult(ls.normal, forceMagnitude);
        appliedForce.add(force);
        forceApplied = true;
        println("CROSSING");
      }
    }

    if (!forceApplied) {
      appliedForce.set(0, 0);
    }

  fEE.set(appliedForce);*/
  
  
  
  PVector appliedForce = new PVector(0, 0);
        boolean forceApplied = false;

        for (LineSegment ls : lineSegments1) {
          if (isNearLineSegment(ls, mappedEE)) {
            //println(mappedEE);
            PVector velocity = PVector.sub(posEE, lastPosEE);
            PVector forceDirection = ls.normal.copy().normalize();
            //PVector forceDirection = velocity.copy().normalize().mult(-1);

            // If velocity is in the same direction as the normal, reverse the force direction
            /*if (PVector.dot(velocity, forceDirection) > 0) {
              forceDirection.mult(-1);
            }*/
            
                // Determine if the point is on the 'inside' or 'outside' of the line segment
            PVector toPoint = PVector.sub(mappedEE, ls.start);
            if (PVector.dot(toPoint, forceDirection) < 0) {
              forceDirection.mult(-1);
            }
                    

            PVector force = PVector.mult(forceDirection, forceMagnitude);
            appliedForce.add(force);
            forceApplied = true;
          }
        }

        if (!forceApplied) {
          appliedForce.set(0, 0);
        }

        fEE.set(appliedForce);


    
  } 

 
  
 //END OF EXPERIENCES**********************************************************************************************************************     
        
      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
    }
    
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

void keyPressed() {
  if(key == '1') {
    ffNum = FFNUM.ONE;
  } else if(key == '2') {
    ffNum = FFNUM.TWO;
  } else if(key == '3') {
    ffNum = FFNUM.THREE;
  }
  else if(key == '5') {
    ffNum = FFNUM.FIVE;
  }
    else if(key == '6') {
    ffNum = FFNUM.SIX;
  }
 else if(key == '4') {
    ffNum = FFNUM.FOUR;
  }
  
 else if(key == '7') {
  ffNum = FFNUM.SEVEN;
}
}

float mapX(float x) {
    // Scale the x-coordinate from the range (-0.07 to 0.055) to (0 to 1000)
    // Formula: newX = (x - oldMin) * (newMax - newMin) / (oldMax - oldMin) + newMin
    return (x + 0.07) * (1000.0 / (0.066 + 0.07));
}

float mapY(float y) {
    // Scale the y-coordinate from the range (0 to 0.1) to (0 to 650)
    // Formula: newY = y * (newMax / oldMax)
    return y * (650.0 / 0.1);
}


// Check if the end-effector crosses the line segment
boolean isCrossingLineSegment(LineSegment ls, PVector position) {
  // Implement line segment crossing logic here
  // This is a simplified version, you might need a more robust algorithm
  PVector closestPoint = closestPointOnLine(ls.start, ls.end, position);
  float distance = PVector.dist(closestPoint, position);
  return distance < threshold;  // Assuming a small threshold for crossing
}

// Find the closest point on a line segment to a given point
PVector closestPointOnLine(PVector a, PVector b, PVector p) {
  transformPoint(a.x,a.y);
  transformPoint(b.x,b.y);
  
  
  PVector ap = PVector.sub(p, a);
  PVector ab = PVector.sub(b, a);
  ab.normalize();
  ab.mult(ap.dot(ab));
  PVector result = PVector.add(a, ab);
  float t = PVector.sub(result, a).dot(PVector.sub(b, a).normalize());
  t = constrain(t, 0, PVector.dist(a, b));
  return PVector.add(a, PVector.sub(b, a).normalize().mult(t));
}


/*
//FOR TRANSFORMED COORDS
// Check if the end-effector crosses the line segment
boolean isCrossingLineSegment(LineSegment ls, PVector position) {
  // Implement line segment crossing logic here
  // This is a simplified version, you might need a more robust algorithm
  PVector closestPoint = closestPointOnLine(ls.start.x/14767.38-0.04947, (ls.start.y*0.07653)/1000+0.02325, ls.end.x/14767.38-0.04947, (ls.end.y*0.07653)/1000+0.02325, position.x, position.y);
  float distance = PVector.dist(closestPoint, position);
  return distance < threshold;  // Assuming a small threshold for crossing
}

// Find the closest point on a line segment to a given point
PVector closestPointOnLine(float ax, float ay, float bx, float by, float px, float py) {
  
  
  float apx = px - ax;
  float apy = py - ay;
  float abx = bx - ax;
  float aby = by - ay;
  
  // Normalize AB
  float abLen = dist(ax, ay, bx, by);
  abx /= abLen;
  aby /= abLen;
  
  // Project AP onto AB
  float ap_dot_ab = apx * abx + apy * aby;
  float proj_abx = abx * ap_dot_ab;
  float proj_aby = aby * ap_dot_ab;
  
  // Calculate the closest point
  float closestX = ax + proj_abx;
  float closestY = ay + proj_aby;
  
  // Clamp the projection to the segment
  float t = ((closestX - ax) * (bx - ax) + (closestY - ay) * (by - ay)) / (abLen * abLen);
  t = constrain(t, 0, 1);
  
  // Calculate the final closest point
  closestX = ax + t * (bx - ax);
  closestY = ay + t * (by - ay);
  
  //println("CLOSEST POINT: (" + closestX + ", " + closestY + ")");
  PVector output = transformPoint(closestX, closestY);
  
  //println("closest point: ");
  //println(output);
  return output;
}
*/

// Distance function to mimic PVector.dist
float distanceCalc(float x1, float y1, float x2, float y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}


PVector transformPoint(float x, float y) {
  float tx = map(x, 0, textureImage.width, 200, 200 + textureImage.width * 0.78);
  float ty = map(y, 0, textureImage.height, 80, 80 + textureImage.height * 0.78);
  return new PVector(tx, ty);
}

PVector transformPoint2(float x, float y) {
  float tx = map(x, 0, textureImage2.width, 200, 200 + textureImage2.width * 0.2);
  float ty = map(y, 0, textureImage2.height, 80, 80 + textureImage2.height * 0.2);
  return new PVector(tx, ty);
}


/*PVector transformPoint(float x, float y) {
  // Transform x coordinate from [0, 750] to [-0.04947, 0.00130]
  float transformedX = ((x / 14767.38 - 0.04947)*0.36)-0.0237;

  // Transform y coordinate from [0, 1000] to [0.02325, 0.09978]
  float transformedY = ((y * 0.07653 / 1000 + 0.02325)*0.7)+0.023468;

  // Map transformed coordinates to the canvas space
  //float mappedX = map(transformedX, -0.04947, 0.00130, 0, width);
  //float mappedY = map(transformedY, 0.09978, 0.02325, height, 0); // Assuming y should be inverted

  return new PVector(transformedX, transformedY);
}*/

/*PVector transformEE(float x, float y) {
  float tx = map(x, min_x, max_x, 0, textureImage.width);
  float ty = map(y, min_y, max_y, 0, textureImage.height);
  return new PVector(tx, ty);
}*/

PVector transformEE(float x, float y) {
  // Map x and y from min_x to max_x, min_y to max_y to the workspace dimensions
  float tx = map(x, min_x, max_x, 0, 1000)-40;
  float ty = map(y, min_y, max_y, 0, 650);
  return new PVector(tx, ty);
}


/*PVector transformEE(float x, float y) {
  // Transform x coordinate from [0, 750] to [-0.04947, 0.00130]
  float transformedX = (1000 * (x + 0.074f)) / 0.123f;

  // Transform y coordinate from [0, 1000] to [0.02325, 0.09978]
  float transformedY = (650 * (y - 0.017f)) / 0.090f;

  // Map transformed coordinates to the canvas space
  //float mappedX = map(transformedX, -0.04947, 0.00130, 0, width);
  //float mappedY = map(transformedY, 0.09978, 0.02325, height, 0); // Assuming y should be inverted

  return new PVector(transformedX, transformedY);
}*/







// Apply force to the end-effector
void applyForceToEffector(PVector force) {
  // This function should send the force to the Haply2diy device
  println("Applying force: " + force);
  // Replace this with the actual code to set the force on the Haply2diy device
  fEE.set(force.x, force.y);
}


boolean isNearLineSegment(LineSegment ls, PVector position) {
  PVector ab = PVector.sub(ls.end, ls.start);
  PVector ap = PVector.sub(position, ls.start);
  float ab_ab = PVector.dot(ab, ab);
  float ap_ab = PVector.dot(ap, ab);
  float t = ap_ab / ab_ab;

  t = constrain(t, 0, 1);  // Clamp t to the [0, 1] range

  PVector closestPoint = PVector.add(ls.start, PVector.mult(ab, t));
  float distance = PVector.dist(closestPoint, position);

  return distance < threshold;
}



/* end helper functions section ****************************************************************************************/
